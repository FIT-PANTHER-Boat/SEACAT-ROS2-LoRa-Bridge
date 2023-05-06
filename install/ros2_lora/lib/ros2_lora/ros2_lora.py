#!/usr/bin/env python3

import rclpy    # Import the ROS2 Python Client library
from rclpy.node import Node     # Import the Node Class from the ROS2 Python client library
from lora_msgs.msg import LoRaMessage   # Import the custom LoRaMessage message type from the lora_msgs package
import busio
import board
import digitalio
from adafruit_rfm9x import RFM9x        # Import the RFM9x class from the adafruit_rfm9x library for LoRa communication
import time     # Import the time module for sleep and delay functions

# Configure the LoRa radio
RADIO_FREQ_MHZ = 915.0  # Set the radio frequency in MHz
CS = digitalio.DigitalInOut(board.D21)  # Chip Select pin for the RFM95 module
RESET = digitalio.DigitalInOut(board.D20)       # Reset pin for the RFM95 module
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)    # SPI interface for the RFM95 module
rfm9x = RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)   # Initialize the RFM95 module 

# Define the ROS2LoRaBridge class that inherits from rclpy.node.Node
class ROS2LoRaBridge(Node):
        def __init__(self):
                super().__init__('ros2_lora_bridge')    # Initialize the base class with the node name
                self.publisher = self.create_publisher(LoRaMessage, 'lora_received', 10)        # Publisher for received LoRa messages
                self.subscription = self.create_subscription(LoRaMessage, 'lora_send', self.send_lora_message, 10)      # Subscriber for messages to be sent via LoRa
                self.timer = self.create_timer(0.5, self.receive_lora_message)  # Create a timer to call receive_lora_message every 0.5 seconds

        # Define the send_lora_message method to send a LoRaMessage via the LoRa radio
        def send_lora_message(self, msg):
                serialized_msg = msg.serialize()        # Serialize the message to be sent
                rfm9x.send(serialized_msg)      # Send the serialized message via LoRa

        # Define the receive_lora_message method to check for and process received LoRa messages
        def receive_lora_message(self):
                packet = rfm9x.receive()        # Receive a LoRa packet (if available)
                if packet is not None:
                        lora_msg = LoRaMessage()
                        lora_msg.deserialize(packet)    # Deserialize the received packet into a LoRaMessage
                        self.publisher.publish(lora_msg)        # Publish the received packet as a ROS2 message

# Define the main function for running the ROS2LoRaBridge node
def main(args=None):
        rclpy.init(args=args)   # initialize the ROS2 Python client library
        ros2_lora_bridge = ROS2LoRaBridge()     # Initialize the ROS2LoRaBridge node 
        executor = rclpy.executors.SingleThreadedExecutor()     # Create a single-threaded executor
        executor.add_node(ros2_lora_bridge)     # Add the node to the executor 

        executor.spin() # Use executor.spin() to handle both ROS2 messages and the LoRa messages

        executor.shutdown()     # Shut down the executor
        ros2_lora_bridge.destroy_node() # Clean up node resources
        rclpy.shutdown()        # Shut down the ROS2 system

# Call the main function to start the ROS2LoRaBridge node
if __name__ == '__main__':
        main()  # Call the main function
