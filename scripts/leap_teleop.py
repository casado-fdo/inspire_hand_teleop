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
# [pinky, ring, middle, index, thumb2, thumb1]
HUMAN_HAND_JOINT_MIN = [95, 85, 85, 85, 135, 85]  # Minimum angles for each joint in degrees
HUMAN_HAND_JOINT_MAX = [160, 165, 165, 170, 170, 125]  # Maximum angles for each joint in degrees
INSPIRE_HAND_JOINT_MIN = [0, 0, 0, 0, 0, 0]  # Minimum angles for Inspire hand joints [0-1000]
INSPIRE_HAND_JOINT_MAX = [1000, 1000, 1000, 1000, 1000, 1000]  # Maximum angles for Inspire hand joints [0-1000]
FIST_ANGLES = [0, 0, 0, 0, 0, 1000]  # Fist angles for Inspire hand
PINCH_INDEX_ANGLES = [-1, -1, -1, 482, 560, 110]  # Pinch angles for Inspire hand (thumb touching index)
PINCH_OTHER_ANGLES = [400, 400, 475, 482, 660, 0]  # Pinch angles for Inspire hand (thumb touching any other finger)
INDEX_PINCH_THRESHOLD = 0.03
MIDDLE_PINCH_THRESHOLD = 0.05
RING_PINCH_THRESHOLD = 0.055
PINKY_PINCH_THRESHOLD = 0.045
FIST_THRESHOLD = 0.19


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
                start_bone_idx = 1 if i == 0 else 1
                # Get the start and end positions of the first and last bones
                first_bone_start = finger.bone_list[start_bone_idx].bone_start
                first_bone_end = finger.bone_list[start_bone_idx].bone_end
                last_bone_end = finger.bone_list[-1].bone_end

                # calculate the angle between the 3 points
                angle = self.get_angle_between_3_points(first_bone_start, first_bone_end, last_bone_end)
                angles[i+1] = angle  # Store the angle for the current finger
            
                if i == 0:  # Thumb is at index 0
                    # For the 2nd DoF of the thumb, we calculate the angle between the thumb's tip and the pinky's first bone
                    pinky_first_bone_start = msg.finger_list[-1].bone_list[0].bone_start
                    # Calculate the angle
                    angle2 = self.get_angle_between_3_points(last_bone_end, first_bone_start, pinky_first_bone_start)
                    angles[0] = angle2 
                    
            # Reverse the angles list to match the Inspire hand joint order 
            angles.reverse()
            mapped_angles = angles
            #rospy.loginfo(f"Raw angles from Leap Motion: {angles}")
            for i, angle in enumerate(angles):
                # Map the angle to the Inspire hand joint range
                mapped_angle = int((angle - HUMAN_HAND_JOINT_MIN[i]) / (HUMAN_HAND_JOINT_MAX[i] - HUMAN_HAND_JOINT_MIN[i]) * (INSPIRE_HAND_JOINT_MAX[i] - INSPIRE_HAND_JOINT_MIN[i]) + INSPIRE_HAND_JOINT_MIN[i])
                mapped_angle = max(INSPIRE_HAND_JOINT_MIN[i], min(INSPIRE_HAND_JOINT_MAX[i], mapped_angle))
                mapped_angles[i] = mapped_angle

            # Fine tune for specific hand gestures
            thumb_tip = msg.finger_list[0].bone_list[-1].bone_end
            index_tip = msg.finger_list[1].bone_list[-1].bone_end
            middle_tip = msg.finger_list[2].bone_list[-1].bone_end
            ring_tip = msg.finger_list[3].bone_list[-1].bone_end
            pinky_tip = msg.finger_list[4].bone_list[-1].bone_end
            palm_center = msg.palm_center.position

            thumb_to_index_distance = math.sqrt(
                (thumb_tip.x - index_tip.x) ** 2 +
                (thumb_tip.y - index_tip.y) ** 2 +
                (thumb_tip.z - index_tip.z) ** 2
            )
            thumb_to_middle_distance = math.sqrt(
                (thumb_tip.x - middle_tip.x) ** 2 +
                (thumb_tip.y - middle_tip.y) ** 2 +
                (thumb_tip.z - middle_tip.z) ** 2
            )
            thumb_to_ring_distance = math.sqrt(
                (thumb_tip.x - ring_tip.x) ** 2 +
                (thumb_tip.y - ring_tip.y) ** 2 +
                (thumb_tip.z - ring_tip.z) ** 2
            )
            thumb_to_pinky_distance = math.sqrt(
                (thumb_tip.x - pinky_tip.x) ** 2 +
                (thumb_tip.y - pinky_tip.y) ** 2 +
                (thumb_tip.z - pinky_tip.z) ** 2
            )
            thumb_to_palm_distance = math.sqrt(
                (thumb_tip.x - palm_center.x) ** 2 +
                (thumb_tip.y - palm_center.y) ** 2 +
                (thumb_tip.z - palm_center.z) ** 2
            )
            index_to_palm_distance = math.sqrt(
                (index_tip.x - palm_center.x) ** 2 +
                (index_tip.y - palm_center.y) ** 2 +
                (index_tip.z - palm_center.z) ** 2
            ) 
            middle_to_palm_distance = math.sqrt(
                (middle_tip.x - palm_center.x) ** 2 +
                (middle_tip.y - palm_center.y) ** 2 +
                (middle_tip.z - palm_center.z) ** 2
            )
            ring_to_palm_distance = math.sqrt(
                (ring_tip.x - palm_center.x) ** 2 +
                (ring_tip.y - palm_center.y) ** 2 +
                (ring_tip.z - palm_center.z) ** 2
            )
            pinky_to_palm_distance = math.sqrt(
                (pinky_tip.x - palm_center.x) ** 2 +
                (pinky_tip.y - palm_center.y) ** 2 +
                (pinky_tip.z - palm_center.z) ** 2
            )
            total_distance_to_palm = (thumb_to_palm_distance + index_to_palm_distance + middle_to_palm_distance + ring_to_palm_distance + pinky_to_palm_distance)
            #rospy.loginfo("Total distance to palm: {:.2f} m".format(total_distance_to_palm))

            # Check for FIST
            if total_distance_to_palm < FIST_THRESHOLD:
                rospy.loginfo_throttle(1, "FIST gesture detected.")
                mapped_angles = FIST_ANGLES  # Set angles for a fist gesture
            # Check for PINCH with the index finger
            elif thumb_to_index_distance < INDEX_PINCH_THRESHOLD and thumb_to_middle_distance > thumb_to_index_distance:
                # Thumb is touching the index finger
                rospy.loginfo_throttle(1, "Thumb is touching the INDEX finger.")
                mapped_angles[3:6] = PINCH_INDEX_ANGLES[3:6]  # Set angles for pinch with index finger
            # Check for PINCH with the middle finger
            elif thumb_to_middle_distance < MIDDLE_PINCH_THRESHOLD and thumb_to_index_distance > thumb_to_middle_distance:
                # Thumb is touching the middle finger
                rospy.loginfo_throttle(1, "Thumb is touching the MIDDLE finger.")
                mapped_angles[2] = PINCH_OTHER_ANGLES[2]
                mapped_angles[3] = max(mapped_angles[3], PINCH_OTHER_ANGLES[3])
                mapped_angles[4:6] = PINCH_OTHER_ANGLES[4:6]
            # Check for PINCH with the ring finger
            elif thumb_to_ring_distance < RING_PINCH_THRESHOLD and thumb_to_middle_distance > thumb_to_ring_distance:
                # Thumb is touching the ring finger
                rospy.loginfo_throttle(1, "Thumb is touching the RING finger.")
                mapped_angles[1] = PINCH_OTHER_ANGLES[1]
                mapped_angles[2:4] = [max(a, b) for a, b in zip(PINCH_OTHER_ANGLES[2:4], mapped_angles[2:4])]
                mapped_angles[4:6] = PINCH_OTHER_ANGLES[4:6]
            # Check for PINCH with the pinky finger
            elif thumb_to_pinky_distance < PINKY_PINCH_THRESHOLD and thumb_to_ring_distance > thumb_to_pinky_distance:
                # Thumb is touching the pinky finger
                rospy.loginfo_throttle(1, "Thumb is touching the PINKY finger.")
                mapped_angles[0] = PINCH_OTHER_ANGLES[0]
                mapped_angles[1:4] = [max(a, b) for a, b in zip(PINCH_OTHER_ANGLES[1:4], mapped_angles[1:4])]
                mapped_angles[4:6] = PINCH_OTHER_ANGLES[4:6]
            # Finally, for SAFETY, make sure that if the index is closed, the thumb is opened, and vice versa, to avoid collisions
            elif mapped_angles[3] < PINCH_INDEX_ANGLES[3]:
                rospy.loginfo_throttle(1,"INDEX finger is closed, opening Thumb's 2nd DoF.")
                if mapped_angles[5] < 600:
                    mapped_angles[4] = max(mapped_angles[4], 350)
                    mapped_angles[5] = 600
                else:
                    mapped_angles[4] = max(mapped_angles[4], 200)


            # Finally, update the last joint positions
            self.last_joint_positions = mapped_angles
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
            else:
                pass # TODO: After some time, move the hand to home position
            # Sleep to maintain the loop rate
            self.rate.sleep()

    
    def get_angle_between_3_points(self, p1, p2, p3):
        """
        Calculate the angle between three points p1, p2, and p3.
        p2 is the vertex of the angle.
        """
        v1 = [p1.x - p2.x, p1.y - p2.y, p1.z - p2.z]
        v2 = [p3.x - p2.x, p3.y - p2.y, p3.z - p2.z]
        return self.get_angle_between_vectors(v1, v2)


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