#!/usr/bin/env python3
import rospy
from leap_motion_controller.msg import Hand
from sensor_msgs.msg import JointState
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

SMOTHING_FACTOR = 0.03 # Smoothing factor for joint angles

class LeapTeleopNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('inspire_teleop', anonymous=True)

        # PARAMETERS
        # Leap Motion and Inspire hand topics 
        self.leap_topic = rospy.get_param('leap_topic', '/leapmotion/hands/right')
        self.hand_topic = rospy.get_param('hand_topic', '/inspire_hand/set_angles')
        
        # Control rate
        self.freq = rospy.get_param('rate', 100)  # Hz
        
        # Human hand joint angle limits (degrees)
        self.human_hand_joint_min = rospy.get_param('human_hand_joint_min', [95, 85, 85, 85, 135, 85])
        self.human_hand_joint_max = rospy.get_param('human_hand_joint_max', [160, 165, 165, 170, 170, 125])

        # Inspire hand joint angle limits (scaled 0-1000)
        self.inspire_hand_joint_min = rospy.get_param('inspire_hand_joint_min', [0, 0, 0, 0, 0, 0])
        self.inspire_hand_joint_max = rospy.get_param('inspire_hand_joint_max', [1000, 1000, 1000, 1000, 1000, 1000])

        # Predefined angles for gestures
        self.fist_angles = rospy.get_param('fist_angles', [0, 0, 0, 0, 0, 1000])
        self.pinch_index_angles = rospy.get_param('pinch_index_angles', [-1, -1, -1, 482, 560, 110])
        self.pinch_other_angles = rospy.get_param('pinch_other_angles', [400, 400, 475, 482, 660, 0])

        # Thresholds for gestures
        self.index_pinch_threshold = rospy.get_param('index_pinch_threshold', 0.03)
        self.middle_pinch_threshold = rospy.get_param('middle_pinch_threshold', 0.05)
        self.ring_pinch_threshold = rospy.get_param('ring_pinch_threshold', 0.055)
        self.pinky_pinch_threshold = rospy.get_param('pinky_pinch_threshold', 0.045)
        self.fist_threshold = rospy.get_param('fist_threshold', 0.19)

        # Hand side (right or left)
        self.hand_side = rospy.get_param('hand_side', 'right')  # Default to right hand

        hand_prefix = 'L' if self.hand_side == 'left' else 'R'
        # Joint names for the right hand
        self.joint_names = [f"{hand_prefix}{suffix}" for suffix in JOINT_NAMES_SUFFIXES]

        # Subscriber to the Leap Motion data topic
        self.leap_sub = rospy.Subscriber(self.leap_topic, Hand, self.leap_callback, queue_size=1)

        # Publisher to the robotic hand joint angles topic
        self.hand_pub = rospy.Publisher(self.hand_topic, JointState, queue_size=1)

        # Initialize joint angles
        self.joint_angles = JointState()
        self.joint_angles.name = self.joint_names
        self.joint_angles.position = [0] * len(self.joint_names)
        self.last_joint_positions = [-1] * len(self.joint_names) 

        self.rate = rospy.Rate(self.freq)  # Set the loop rate

        rospy.loginfo("Inspire Hand Teleop node ready!")


    def leap_callback(self, msg):
        """
        Callback function to process Leap Motion data and update joint angles.
        """
        angles = [0] * len(self.joint_names)  # Initialize angles for each joint
        try:
            # 1. DIRECT MAPPING from Leap Motion data
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
                mapped_angle = int((angle - self.human_hand_joint_min[i]) / (self.human_hand_joint_max[i] - self.human_hand_joint_min[i]) * (self.inspire_hand_joint_max[i] - self.inspire_hand_joint_min[i]) + self.inspire_hand_joint_min[i])
                mapped_angle = max(self.inspire_hand_joint_min[i], min(self.inspire_hand_joint_max[i], mapped_angle))
                mapped_angles[i] = mapped_angle

            # 2. FINE TUNING of the angles based on finger distances
            thumb_tip = msg.finger_list[0].bone_list[-1].bone_end
            index_tip = msg.finger_list[1].bone_list[-1].bone_end
            middle_tip = msg.finger_list[2].bone_list[-1].bone_end
            ring_tip = msg.finger_list[3].bone_list[-1].bone_end
            pinky_tip = msg.finger_list[4].bone_list[-1].bone_end
            palm_center = msg.palm_center.position

            thumb_to_index_distance = self.euclidean_distance(thumb_tip, index_tip)
            thumb_to_middle_distance = self.euclidean_distance(thumb_tip, middle_tip)
            thumb_to_ring_distance = self.euclidean_distance(thumb_tip, ring_tip)
            thumb_to_pinky_distance = self.euclidean_distance(thumb_tip, pinky_tip)
            thumb_to_palm_distance = self.euclidean_distance(thumb_tip, palm_center)
            index_to_palm_distance = self.euclidean_distance(index_tip, palm_center)
            middle_to_palm_distance = self.euclidean_distance(middle_tip, palm_center)
            ring_to_palm_distance = self.euclidean_distance(ring_tip, palm_center)
            pinky_to_palm_distance = self.euclidean_distance(pinky_tip, palm_center)
            total_distance_to_palm = (thumb_to_palm_distance + index_to_palm_distance + middle_to_palm_distance + ring_to_palm_distance + pinky_to_palm_distance)
            #rospy.loginfo("Total distance to palm: {:.2f} m".format(total_distance_to_palm))

            # Check for FIST
            if total_distance_to_palm < self.fist_threshold:
                rospy.loginfo_throttle(1, "FIST gesture detected.")
                mapped_angles = self.fist_angles  # Set angles for a fist gesture
                # mapped_angles = [int(SMOTHING_FACTOR * fist + (1 - SMOTHING_FACTOR) * last) for fist, last in zip(self.fist_angles, self.last_joint_positions)]
            # Check for PINCH with the index finger
            elif thumb_to_index_distance < self.index_pinch_threshold and thumb_to_middle_distance > thumb_to_index_distance:
                # Thumb is touching the index finger
                rospy.loginfo_throttle(1, "Thumb is touching the INDEX finger.")
                mapped_angles[3:6] = self.pinch_index_angles[3:6]  # Set angles for pinch with index finger
            # Check for PINCH with the middle finger
            elif thumb_to_middle_distance < self.middle_pinch_threshold and thumb_to_index_distance > thumb_to_middle_distance and middle_to_palm_distance > thumb_to_middle_distance:
                # Thumb is touching the middle finger
                rospy.loginfo_throttle(1, "Thumb is touching the MIDDLE finger.")
                mapped_angles[2] = self.pinch_other_angles[2]
                mapped_angles[3] = max(mapped_angles[3], self.pinch_other_angles[3])
                mapped_angles[4:6] = self.pinch_other_angles[4:6]
            # Check for PINCH with the ring finger
            elif thumb_to_ring_distance < self.ring_pinch_threshold and thumb_to_middle_distance > thumb_to_ring_distance:
                # Thumb is touching the ring finger
                rospy.loginfo_throttle(1, "Thumb is touching the RING finger.")
                mapped_angles[1] = self.pinch_other_angles[1]
                mapped_angles[2:4] = [max(a, b) for a, b in zip(self.pinch_other_angles[2:4], mapped_angles[2:4])]
                mapped_angles[4:6] = self.pinch_other_angles[4:6]
            # Check for PINCH with the pinky finger
            elif thumb_to_pinky_distance < self.pinky_pinch_threshold and thumb_to_ring_distance > thumb_to_pinky_distance:
                # Thumb is touching the pinky finger
                rospy.loginfo_throttle(1, "Thumb is touching the PINKY finger.")
                mapped_angles[0] = self.pinch_other_angles[0]
                mapped_angles[1:4] = [max(a, b) for a, b in zip(self.pinch_other_angles[1:4], mapped_angles[1:4])]
                mapped_angles[4:6] = self.pinch_other_angles[4:6]
            # Finally, for SAFETY, make sure that if the index is closed, the thumb is opened, and vice versa, to avoid collisions
            elif mapped_angles[3] < self.pinch_index_angles[3]:
                rospy.loginfo_throttle(1,"INDEX finger is closed, opening Thumb's 2nd DoF.")
                if mapped_angles[5] < 600:
                    mapped_angles[4] = max(mapped_angles[4], 350)
                    mapped_angles[5] = 600
                else:
                    mapped_angles[4] = max(mapped_angles[4], 200)


            # Finally, update the last joint positions
            self.last_joint_positions =  [int(SMOTHING_FACTOR * fist + (1 - SMOTHING_FACTOR) * last) for fist, last in zip(mapped_angles, self.last_joint_positions)]
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

    def euclidean_distance(self, point1, point2):
        """
        Calculate the Euclidean distance between two points in 3D space.
        """
        return math.sqrt(
            (point1.x - point2.x) ** 2 +
            (point1.y - point2.y) ** 2 +
            (point1.z - point2.z) ** 2
        )

if __name__ == '__main__':
    try:
        node = LeapTeleopNode()
        node.run()
    except rospy.ROSInterruptException:
        pass