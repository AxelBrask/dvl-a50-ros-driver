#!/usr/bin/env python3

import rospy

from waterlinked_a50_ros_driver.msg import DVL as waterlinkedDVLmsg
from waterlinked_a50_ros_driver.msg import DVLBeam as waterlinkedDVLbeamMsg

from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolResponse
from std_srvs.srv import SetBoolRequest

from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Bool 

from smarc_msgs.msg import DVLBeam
from smarc_msgs.msg import DVL

import socket
import json
from time import sleep
import select
import threading

class DVLDriver(object):
    
	def __init__(self):
		
		self.dvl_frame = rospy.get_param('~dvl_frame', 'sam/dvl_link')
		self.dvl_topic = rospy.get_param('~dvl_topic', '/sam/core/dvl')
		self.dvl_ctrl_srv = rospy.get_param('~dvl_on_off_srv', 'core/toggle_dvl')
		self.dvl_raw_topic = rospy.get_param('~dvl_raw_topic', 'dvl_raw_output')
		self.relay_topic = rospy.get_param('~relay_topic', 'sam/core/dvl_relay')
  
		self.pub_raw = rospy.Publisher( self.dvl_raw_topic, String, queue_size=10)
		self.pub_relay = rospy.Publisher( self.relay_topic, Bool, queue_size=10)

		# Waterlinked parameters
		self.TCP_IP = rospy.get_param("~ip", "10.42.0.186")
		self.TCP_PORT = rospy.get_param("~port", 16171)
		self.do_log_raw_data = rospy.get_param("~do_log_raw_data", False)    
  
		# Waterlinked variables
		self.s = None
		self.dvl_on = False
		self.oldJson = ""
  
  		# Topics for debugging
		self.dvl_en_pub = rospy.Publisher('dvl_enable', Bool, queue_size=10)
  
		# Service to start/stop DVL and DVL data publisher
		self.switch_srv = rospy.Service(self.dvl_ctrl_srv, SetBool, self.dvl_switch_cb) 
		self.dvl_pub = rospy.Publisher(self.dvl_topic, DVL, queue_size=10)
		self.switch = False

		self.data_buffer = b""
  
		# Connect to the DVL and turn it off
		while not self.connect():
			rospy.logerr("Coud not connect, retrying in 5 seconds...")
			rospy.sleep(5)

		socket_thread = threading.Thread(target=self.handle_socket, args=(self.s,))
		socket_thread.start()
	
		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			rate.sleep()

		self.send_relay_msg(relay=False)
		self.close()
	
	def handle_socket(self,sock):
		while True:
			try:
				data = sock.recv(1024)
				if not data:  # If data is empty, the remote end has closed the socket
					# sock.close()
					break
				self.data_buffer += data
				while b'\n' in self.data_buffer:
					line, self.data_buffer = self.data_buffer.split(b'\n', 1)
					self.receive_dvl(line.decode('utf-8'))
			except socket.timeout as err:
				rospy.logerr("No data received? {}".format(err))
			except socket.error as err:
				self.connect()
				sock = self.s
				rospy.sleep(2)


   
	def receive_dvl(self, raw_data):
     
		theDVL = DVL()
		beam0 = DVLBeam()
		beam1 = DVLBeam()
		beam2 = DVLBeam()
		beam3 = DVLBeam()
  
		# raw_data = self.getData()
		data = json.loads(raw_data)

		# edit: the logic in the original version can't actually publish the raw data
		# we slightly change the if else statement so now
		# do_log_raw_data is true: publish the raw data to /dvl/json_data topic, fill in theDVL using velocity data and publish to dvl/data topic
		# do_log_raw_data is true: only fill in theDVL using velocity data and publish to dvl/data topic

		if self.do_log_raw_data:
			rospy.loginfo(raw_data)
			self.pub_raw.publish(raw_data)
			if data["type"] != "velocity":
				return
		else:
			if data["type"] != "velocity":
				return
			self.pub_raw.publish(raw_data)

		theDVL.header.stamp = rospy.Time.now()
		theDVL.header.frame_id = self.dvl_frame

		theDVL.velocity.x = data["vx"]
		theDVL.velocity.y = data["vy"]
		theDVL.velocity.z = data["vz"]
		theDVL.velocity_covariance[0] = data["covariance"][0][0]
		theDVL.velocity_covariance[4] = data["covariance"][1][1]
		theDVL.velocity_covariance[8] = data["covariance"][2][2]
		theDVL.altitude = data["altitude"]

		# Todo : Add beam covariances (not available for waterlinked)

		beam0.range = data["transducers"][0]["distance"]
		beam0.velocity = data["transducers"][0]["velocity"]

		beam1.range = data["transducers"][1]["distance"]
		beam1.velocity = data["transducers"][1]["velocity"]

		beam2.range = data["transducers"][2]["distance"]
		beam2.velocity = data["transducers"][2]["velocity"]

		beam3.range = data["transducers"][3]["distance"]
		beam3.velocity = data["transducers"][3]["velocity"]


		# theDVL.header.stamp = rospy.Time.now()
		# theDVL.header.frame_id = self.dvl_frame
		# theDVL.time = data["time"]
		# theDVL.velocity.x = data["vx"]
		# theDVL.velocity.y = data["vy"]
		# theDVL.velocity.z = data["vz"]
		# theDVL.fom = data["fom"]
		# theDVL.altitude = data["altitude"]
		# theDVL.velocity_valid = data["velocity_valid"]
		# theDVL.status = data["status"]
		# theDVL.form = data["format"]

		# beam0.id = data["transducers"][0]["id"]
		# beam0.velocity = data["transducers"][0]["velocity"]
		# beam0.distance = data["transducers"][0]["distance"]
		# beam0.rssi = data["transducers"][0]["rssi"]
		# beam0.nsd = data["transducers"][0]["nsd"]
		# beam0.valid = data["transducers"][0]["beam_valid"]

		# beam1.id = data["transducers"][1]["id"]
		# beam1.velocity = data["transducers"][1]["velocity"]
		# beam1.distance = data["transducers"][1]["distance"]
		# beam1.rssi = data["transducers"][1]["rssi"]
		# beam1.nsd = data["transducers"][1]["nsd"]
		# beam1.valid = data["transducers"][1]["beam_valid"]

		# beam2.id = data["transducers"][2]["id"]
		# beam2.velocity = data["transducers"][2]["velocity"]
		# beam2.distance = data["transducers"][2]["distance"]
		# beam2.rssi = data["transducers"][2]["rssi"]
		# beam2.nsd = data["transducers"][2]["nsd"]
		# beam2.valid = data["transducers"][2]["beam_valid"]

		# beam3.id = data["transducers"][3]["id"]
		# beam3.velocity = data["transducers"][3]["velocity"]
		# beam3.distance = data["transducers"][3]["distance"]
		# beam3.rssi = data["transducers"][3]["rssi"] 
		# beam3.nsd = data["transducers"][3]["nsd"]
		# beam3.valid = data["transducers"][3]["beam_valid"]

		theDVL.beams = [beam0, beam1, beam2, beam3]

		rospy.loginfo_throttle(5,f"[DVL Driver] Valid measurement : {data['velocity_valid']}") 
		self.dvl_pub.publish(theDVL)
        
	def dvl_switch_cb(self, switch_msg : SetBoolRequest):

		rospy.loginfo(f"[DVL Driver] Enable DVL : {switch_msg.data}")
		self.switch = True

		res = SetBoolResponse()
		self.dvl_on = switch_msg.data		
  
		if switch_msg.data:
			self.send_relay_msg(relay=True)
		else:
			self.send_relay_msg(relay=False)
			self.close()

		self.dvl_en_pub.publish(Bool(self.dvl_on))

		res.success = True
		res.message = "Relay message has been sent"

		return res 
        

	def connect(self) -> bool:
		"""
		Connect to the DVL
		"""	

		self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.s.settimeout(1.0)

		rospy.loginfo("[DVL Driver] Trying to connect to DVL")
		return_val = False

		try:
			self.s.connect((self.TCP_IP, self.TCP_PORT))
			return_val = True
			rospy.loginfo("[DVL Driver] Successfully connected")
		except socket.error as err:
			rospy.logerr("No route to host, DVL might be booting? {}".format(err))

		return return_val
	
	def send_relay_msg(self, relay):

		rospy.loginfo(f"[DVL Driver] Setting relay : {relay}")

		relay_msg = Bool()
		relay_msg.data = relay

		self.pub_relay.publish(relay_msg)
   
	def close(self) -> bool:
		"""
		Close the connection to the DVL
		"""
  
		try:
			self.s.close()
			return True
		except Exception as e:
			rospy.logerr("No route to host, DVL might be booting? {}".format(e))
			return False

	def getData(self):
		raw_data = b''

		while not b'\n' in raw_data:
			try:
				rec = self.s.recv(1) # Add timeout for that
				if len(rec) == 0:
					rospy.logerr("Socket closed by the DVL, reopening")
					self.connect()
					# return
					continue
			except socket.timeout as err:
				rospy.logerr("Lost connection with the DVL, reinitiating the connection: {}".format(err))
				self.connect()
				# return
				continue
			raw_data = raw_data + rec
		raw_data = raw_data.decode("utf-8") 
		raw_data = self.oldJson + raw_data
		self.oldJson = ''
		raw_data = raw_data.split('\n')
		self.oldJson = raw_data[1]
		raw_data = raw_data[0]
		return raw_data

	def set_config(self,
		speed_of_sound = None,
		mounting_rotation_offset = None,
		acoustic_enabled = None,
		dark_mode_enabled = None, 
		range_mode = None):
     
		"""
		Set the configuration of the DVL
  
		Values not set will be left blank and will not be changed
		"""
  
  		# Validate input
		valid_input = True
		if speed_of_sound is not None and (speed_of_sound < 0 ):
			rospy.logerr("Invalid speed of sound")
			valid_input = False
		if mounting_rotation_offset is not None and (mounting_rotation_offset < 0 or mounting_rotation_offset > 360):
			rospy.logerr("Invalid mounting rotation offset")
			valid_input = False
		if acoustic_enabled is not None and (acoustic_enabled != "y" and acoustic_enabled != "n"):
			rospy.logerr("Invalid acoustic enabled value")
			valid_input = False 
		if dark_mode_enabled is not None and (dark_mode_enabled != "y" and dark_mode_enabled != "n"):
			rospy.logerr("Invalid dark mode enabled value")
			valid_input = False
		if range_mode is not None and (range_mode != "auto"):
			rospy.logerr("Invalid range mode")
			valid_input = False
   
		if not valid_input:
			return False
     
		paramaters = [speed_of_sound, mounting_rotation_offset, acoustic_enabled, dark_mode_enabled, range_mode]
		command_string = "wcs"
		for i, val in enumerate(paramaters):
      
			command_string +=","
			command_string+= str(val) if val is not None else ""
   
		command_string += "\n"

		rospy.loginfo("[DVL Driver] Sending command: {}".format(command_string))
		self.s.send(command_string.encode())

if __name__ == '__main__':
	rospy.init_node('DVLDriver')
	rospy.loginfo('[DVL Driver] Starting DVL driver')
	try:
		DVLDriver()
	except rospy.ROSInterruptException:
		pass