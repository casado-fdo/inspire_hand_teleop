#!/usr/bin/env python3
import rospy
from leap_motion_controller.msg import Hand
from sensor_msgs.msg import JointState


# Finger joint names
HAND_SIDE = "R"  # "R" for right, "L" for left TODO: Make this configurable via ROS parameter
JOINT_NAMES = [
    f"{HAND_SIDE}_pinky_MCP_joint",
    f"{HAND_SIDE}_ring_MCP_joint",
    f"{HAND_SIDE}_middle_MCP_joint",
    f"{HAND_SIDE}_index_MCP_joint",
    f"{HAND_SIDE}_thumb_MCP_joint2",
    f"{HAND_SIDE}_thumb_MCP_joint1"
]
HAND_JOINT_MIN = 0
HAND_JOINT_MAX = 1000

class LeapTeleopNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('inspire_teleop', anonymous=True)

        # Get parameters 
        self.leap_topic = rospy.get_param('~leap_topic', '/leapmotion/hands/right')
        self.hand_topic = rospy.get_param('~hand_topic', '/inspire_hand/set_angles')
        self.freq = rospy.get_param('~rate', 100)  # Hz

        # Subscriber to the Leap Motion data topic
        self.leap_sub = rospy.Subscriber(self.leap_topic, Hand, self.leap_callback, queue_size=1)

        # Publisher to the robotic hand joint angles topic
        self.hand_pub = rospy.Publisher(self.hand_topic, JointState, queue_size=1)

        # Initialize joint angles
        self.joint_angles = JointState()
        self.joint_angles.name = JOINT_NAMES
        self.joint_angles.position = [0] * len(JOINT_NAMES)
        self.last_joint_positions = [-1] * len(JOINT_NAMES) 

        self.rate = rospy.Rate(self.freq)  # Set the loop rate

        rospy.loginfo("Inspire Hand Teleop node ready!")


    def leap_callback(self, msg):
        """
        Callback function to process Leap Motion data and update joint angles.
        """
        try:
            test = (1 - msg.grab_strength) * 1000
            rospy.loginfo(f"Received Leap Motion data: Grab strength = {msg.grab_strength}, Test value = {test}")

            # Update the last joint positions using a window average
            for i in range(len(self.last_joint_positions)-1):
                self.last_joint_positions[i] = test #self.last_joint_positions[i] * 0.95 + test * 0.05
                self.last_joint_positions[i] = max(HAND_JOINT_MIN, min(HAND_JOINT_MAX, self.last_joint_positions[i]))
            self.last_joint_positions[5] = 1000
            rospy.loginfo(f"Setting Pinky MCP joint angle to: {test}")

        except IndexError:
            rospy.logerr("Leap Motion data does not contain enough values.")


    def run(self):
        while not rospy.is_shutdown():
            if any(position != -1 for position in self.last_joint_positions):
                # Update the joint angles message
                self.joint_angles.header.stamp = rospy.Time.now()
                self.joint_angles.position = self.last_joint_positions

                # Publish the joint angles
                self.hand_pub.publish(self.joint_angles)

                # Sleep to maintain the loop rate
                self.rate.sleep()
            else:
                pass


if __name__ == '__main__':
    try:
        node = LeapTeleopNode()
        node.run()
    except rospy.ROSInterruptException:
        pass