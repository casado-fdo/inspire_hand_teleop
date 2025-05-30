#!/usr/bin/env python3
import rospy
from leap_motion_controller.msg import Hand
from sensor_msgs.msg import JointState
import math


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
HUMAN_HAND_JOINT_MIN = [85, 75, 70, 70, 1, 1]  # Minimum angles for each joint in degrees
HUMAN_HAND_JOINT_MAX = [160, 165, 165, 165, 180, 180]  # Maximum angles for each joint in degrees
INSPIRE_HAND_JOINT_MIN = [0, 0, 0, 0, 0, 0]  # Minimum angles for Inspire hand joints [0-1000]
INSPIRE_HAND_JOINT_MAX = [1000, 1000, 1000, 1000, 1000, 1000]  # Maximum angles for Inspire hand joints [0-1000]

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
        angles = [0] * len(JOINT_NAMES)  # Initialize angles for each joint
        try:
            # Reversed loop to process from pinky to thumb, matching the Inspire hand joint order
            for i, finger in list(enumerate(msg.finger_list)): 
                if i == 0:  # Thumb is at index 0
                    pass  # Skip the thumb for now, as it has a different structure
                    angles[0] = 180
                    angles[1] = 180
                else:
                    # Get the start and end positions of the first and last bones
                    first_bone_start = finger.bone_list[0].bone_start
                    first_bone_end = finger.bone_list[0].bone_end
                    last_bone_end = finger.bone_list[-1].bone_end

                    # calculate the angle between the 3 points
                    v1 = [first_bone_start.x - first_bone_end.x, first_bone_start.y - first_bone_end.y, first_bone_start.z - first_bone_end.z]
                    v2 = [last_bone_end.x - first_bone_end.x, last_bone_end.y - first_bone_end.y, last_bone_end.z - first_bone_end.z]
                    angle = self.get_angle_between_vectors(v1, v2)
                    angles[i+1] = angle  # Store the angle for the current finger

            # Reverse the angles list to match the Inspire hand joint order 
            angles.reverse()
            #rospy.loginfo(f"Raw angles from Leap Motion: {angles}")
            for i, angle in enumerate(angles):
                # Map the angle to the Inspire hand joint range
                mapped_angle = int((angle - HUMAN_HAND_JOINT_MIN[i]) / (HUMAN_HAND_JOINT_MAX[i] - HUMAN_HAND_JOINT_MIN[i]) * (INSPIRE_HAND_JOINT_MAX[i] - INSPIRE_HAND_JOINT_MIN[i]) + INSPIRE_HAND_JOINT_MIN[i])
                mapped_angle = max(INSPIRE_HAND_JOINT_MIN[i], min(INSPIRE_HAND_JOINT_MAX[i], mapped_angle))
                self.last_joint_positions[i] = mapped_angle

            rospy.loginfo(f"Updated joint positions: {self.last_joint_positions}")

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


    def get_angle_between_vectors(self, v1, v2):
        """
        Calculate the angle between two vectors (in degrees).
        """
        dot_product = sum(a * b for a, b in zip(v1, v2))
        magnitude_v1 = math.sqrt(sum(a ** 2 for a in v1))
        magnitude_v2 = math.sqrt(sum(b ** 2 for b in v2))
        if magnitude_v1 == 0 or magnitude_v2 == 0:
            return 0.0
        cos_angle = dot_product / (magnitude_v1 * magnitude_v2)
        return math.acos(cos_angle) * (180.0 / math.pi)


if __name__ == '__main__':
    try:
        node = LeapTeleopNode()
        node.run()
    except rospy.ROSInterruptException:
        pass