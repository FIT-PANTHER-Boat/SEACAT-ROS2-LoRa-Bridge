#!/usr/bin/env python3

import rclpy	# Import the ROS2 Python Client library
from rclpy.node import Node	# Import the Node Class from the ROS2 Python client library
from lora_msgs.msg import LoRaMessage	# Import the custom LoRaMessage message type from the lora_msgs package
import pigpio
from adafruit_rfm9x import RFM9x
import board
import digitalio

# Set the radio frequency and Chip Select pin
RADIO_FREQ_MHZ = 900.0	# Set the radio frequency in MHz
CS = digitalio.DigitalInOut(board.D21)	# Chip Select pin for the RFM95 module 

# Define the ROS2LoRaBridge class that inherits from the Node class
class ROS2LoRaBridge(Node):

	def __init__(self):
		super().__init__('ros2_lora_bridge')	# Initialize the base class with the node name
		self.pi = pigpio.pi()	# Instantiate the pigpio library object
		self.pi.set_mode(CS, pigpio.OUTPUT)	# Set the Chip Slect pin as an output pin
		self.rfm9x = self.init_rfm9x()		# Initialize the RFM9x module
		self.publisher = self.create_publisher(LoRaMessage, 'lora_received', 10)	# Publisher for received LoRa messages
		self.subscription = self.create_subscription(LoRaMessage, 'lora_send', self.send_lora_message, 10)	# Subscriber for messages to be sent via LoRa
		self.timer = self.create_timer(0.5, self.receive_lora_message)	# Create a timer to call receive_lora_message every 0.5 seconds

	# Define the init_rfm9x method to initialize the RFM9x module
	def init_rfm9x(self):
		RESET = 25 	# Set the RESET pin as an output pin
		self.pi.set_mode(RESET, pigpio.OUTPUT)
		spi = self.pi.spi_open(0, 500000, 256)	# Open an SPI channel with 500 Kbps speed
		rfm9x = RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)	# Initialize the RFM9x module with the SPI channel, Chip Select pin, RESET pin, and radio frequency
		rfm9x.baudrate = 1000000		# Baudrate
		rfm9x.tx_power = 23 		# Output power level
		return rfm9x

	# Define the send_lora_message method to send a LoRaMessage via the LoRa radio
	def send_lora_message(self, msg):
		serialized_msg = msg.serialize()	# Serialize the message to be sent
		self.rfm9x.send(serialized_msg)	# Send the serialized message via LoRa

	# Define the receive_lora_message method to check for and process received LoRa messages
	def receive_lora_message(self):
		packet = self.rfm9x.receive()	# Receive a LoRa packet (if available)
		if packet is not None:
			lora_msg = LoRaMessage()
			lora_msg.deserialize(packet)	# Deserialize the received packet into a LoRaMessage
			self.publisher.publish(lora_msg)	# Publish the received packet as a ROS2 message

	# Define the cleanup method to stope the pigpio library object
	def cleanup(self):
		self.pi.stop()
# Define the main function for running the ROS2LoRaBridge node
def main(args=None):
	rclpy.init(args=args)	# initialize the ROS2 Python client library
	ros2_lora_bridge = ROS2LoRaBridge()	# Initialize the ROS2LoRaBridge node

	# Use a try-except block to handle KeyboardInterrupt exceptions
	try:
		rclpy.spin(ros2_lora_bridge)	# Spin the ROS2LoRa node to process ROS2 messages and LoRa messages
	except KeyboardInterrupt:
		print('Shutting down...')
	finally:
		ros2_lora_bridge.cleanup()	# Call the cleanup method to stop the pigpio library object
		ros2_lora_bridge.destroy_node()	# Destroy the node
		rclpy.shutdown()	# Shut down the ROS2 system

# Call the main function to start the ROS2LoRaBridge node
if __name__ == '__main__':
	main()	# Call the main function
