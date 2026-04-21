#!/usr/bin/env python3
import rospy
from leap_motion_controller.msg import Hand
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
import math

'''
This script defines a ROS node that processes Leap Motion data to control the Inspire-Robots hand.
It provides direct mapping with some fine-tuning based on finger distances and predefined gestures.
'''

# Finger joint names suffixes
JOINT_NAMES_SUFFIXES = [
    "_pinky_MCP_joint",
    "_ring_MCP_joint",
    "_middle_MCP_joint",
    "_index_MCP_joint",
    "_thumb_MCP_joint2",
    "_thumb_MCP_joint1"
]

class InspireHandTeleop:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('inspire_teleop', anonymous=True)

        # PARAMETERS
        # Hand side (right or left)
        self.hand_side = rospy.get_param('hand_side', 'right')  # Default to right hand

        # Teleoperation parameters and Inspire hand topics
        self.teleop_source = rospy.get_param('teleop_source')  # 'leap_teleop' or 'lerobot'
        self.hand_ns = rospy.get_param('hand_ns', '/inspire_hand')
        self.hand_state_topic = self.hand_ns + '/hand_state'
        self.hand_control_topic = self.hand_ns + '/set_angles'
        self.inactivity_timeout = rospy.get_param("inactivity_timeout") # s (-1 for infinite)
        rospy.loginfo(f"Using teleoperation source: {self.teleop_source}")

        # Control rate
        self.freq = rospy.get_param('rate', 100)  # Hz

        # Joint state feedback error margin
        self.joint_error_margin = rospy.get_param('joint_feedback_error_margin', 10)

        # Predefined angles for gestures
        self.home_angles = rospy.get_param('fist_angles', [1000, 1000, 1000, 1000, 1000, 1000])
        self.ready_angles = rospy.get_param('open_angles', [1000, 1000, 1000, 1000, 1000, 1000])

        # Force threshold
        self.force_threshold = rospy.get_param('force_threshold', 200)

        hand_prefix = 'L' if self.hand_side == 'left' else 'R'
        self.joint_names = [f"{hand_prefix}{suffix}" for suffix in JOINT_NAMES_SUFFIXES]    

        # Subscriber to the current Hand state
        self.hand_state_sub = rospy.Subscriber(self.hand_state_topic, Int32MultiArray, self.hand_state_callback, queue_size=1)

        # Subscribe to the teleop source
        self.sub_hand_teleop = rospy.Subscriber(self.hand_ns + '/' + self.teleop_source + '/hand_angles', Int32MultiArray, self.hand_teleop_callback, queue_size=1)

        # Publisher to the robotic hand joint angles topic
        self.hand_pub = rospy.Publisher(self.hand_control_topic, JointState, queue_size=1)

        # Initialize joint angles
        self.joint_angles = JointState()
        self.joint_angles.name = self.joint_names
        self.joint_angles.position = [0] * len(self.joint_names)
        self.last_teleop_positions = [1000] * len(self.joint_names) # Initialize last leap positions to a high value (fully open)
        self.last_joint_positions = [-1] * len(self.joint_names)  # Initialize last joint positions
        self.last_force_readings = [0] * len(self.joint_names)    # Initialize last hand state
        self.last_command = [-1] * len(self.joint_names)          # Initialize last command positions
        self.last_command_time = None
        self.grasping = [False] * len(self.joint_names)           # Initialize grasping state for each joint

        self.rate = rospy.Rate(self.freq)  # Set the loop rate

        rospy.loginfo("Inspire Hand Teleop node ready!")
        
        # Register shutdown function
        rospy.on_shutdown(self.shutdown)

    
    def shutdown(self):
        """
        Shutdown function to ensure the hand goes to home position
        """
        rospy.loginfo("Shutting down - moving hand to home position")
        self.go_home()
    
    def hand_state_callback(self, msg):
        """
        Callback function to process the current hand state and update last force readings.
        """
        self.last_joint_positions = msg.data[:6]  # The first 6 values are joint positions
        self.last_force_readings = msg.data[-6:]  # The last 6 values are force readings
        #rospy.loginfo(f"Last force readings: {self.last_force_readings}")


    def hand_teleop_callback(self, msg):
        """
        Callback function to process the teleop hand angles from the selected source.
        """
        self.last_teleop_positions = msg.data
        # rospy.loginfo(f"Received teleop hand angles: {self.last_teleop_positions}")


    def run(self):
        # Wait for a bit
        rospy.sleep(4.5)
        # Initialise the hand from home position
        self.go_home()
        # Wait for a bit
        rospy.sleep(4.5)
        # Now go to ready position
        self.go_ready()
        # Ready to run the teleoperation
        while not rospy.is_shutdown():
            if self.last_command_time != None and self.inactivity_timeout > 0 and (rospy.Time.now() - self.last_command_time).to_sec() > self.inactivity_timeout:
                rospy.loginfo("Inactivity timeout reached. Going to home position.")
                self.go_ready()
                self.last_command_time = None
            elif self.last_teleop_positions is not None and any(position != -1 for position in self.last_teleop_positions):
                # Reset the grasping state only when the new position is fully open for all joints
                if all(position >= (1000 - 15 * self.joint_error_margin) for position in self.last_teleop_positions) and any(self.grasping):
                    self.grasping = [False] * len(self.joint_names)
                    rospy.loginfo("Grasping finished.")
                filtered_joint_positions = []
                for i, position in enumerate(self.last_teleop_positions):
                    # Set to -1 any joint that exceeds the force threshold and the last_leap_position is bigger than last_joint_positions
                    if position == -1 or self.grasping[i]:
                        filtered_joint_positions.append(-1)
                    elif self.last_force_readings[i] > self.force_threshold and position < self.last_joint_positions[i] and self.grasping[i] == False:
                        filtered_joint_positions.append(self.last_joint_positions[i] - 2 * self.joint_error_margin)
                        self.grasping[i] = True
                    else:
                        filtered_joint_positions.append(int(position))

                if any(self.grasping):
                    filtered_joint_positions[5] = -1 # Always ignore the thumb's 2nd DoF when grasping

                # Set to -1 any joint that has not changed significantly from the last command
                for i in range(len(filtered_joint_positions)):
                    if filtered_joint_positions[i] != -1 and self.last_command[i] != -1:
                        if abs(filtered_joint_positions[i] - self.last_command[i]) < self.joint_error_margin:
                            filtered_joint_positions[i] = -1

                # Update the joint angles message
                time_now = rospy.Time.now()
                self.joint_angles.header.stamp = time_now
                self.joint_angles.position = filtered_joint_positions

                # Publish the joint angles
                self.hand_pub.publish(self.joint_angles)
                for i in range(len(self.joint_names)):
                    if filtered_joint_positions[i] != -1:
                        self.last_command[i] = filtered_joint_positions[i]

                self.last_teleop_positions = None
                self.last_command_time = time_now


            # Sleep to maintain the loop rate
            self.rate.sleep()


    def go_home(self):
        """
        Move the hand to the home position (fully closed)
        """
        self.joint_angles.position = self.home_angles
        self.hand_pub.publish(self.joint_angles)

    def go_ready(self):
        """
        Move the hand to the ready position (fully open)
        """
        self.joint_angles.position = self.ready_angles
        self.hand_pub.publish(self.joint_angles)

if __name__ == '__main__':
    try:
        node = InspireHandTeleop()
        node.run()
    except rospy.ROSInterruptException:
        pass