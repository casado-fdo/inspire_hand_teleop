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

class LeapToInspireHand:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('inspire_teleop', anonymous=True)

        # PARAMETERS
        # Hand side (right or left)
        self.hand_side = rospy.get_param('hand_side', 'right')  # Default to right hand

        # Inspire hand topics 
        self.leap_ns = rospy.get_param('leap_ns', '/leapmotion')
        self.leap_topic = self.leap_ns + '/hands/' + self.hand_side
        self.hand_ns = rospy.get_param('hand_ns', '/inspire_hand')
        self.hand_control_topic = self.hand_ns + '/set_angles'
        self.hand_state_topic = self.hand_ns + '/hand_state'

        # Control rate
        self.freq = rospy.get_param('rate', 100)  # Hz

        # Leap smoothing factor
        self.smoothing_factor = rospy.get_param('smoothing_factor', 0.2)

        # Joint state feedback error margin
        self.joint_error_margin = rospy.get_param('joint_feedback_error_margin', 10)

        # Human hand joint angle limits (degrees)
        self.human_hand_joint_min = rospy.get_param('human_hand_joint_min', [95, 85, 85, 85, 135, 95])
        self.human_hand_joint_max = rospy.get_param('human_hand_joint_max', [160, 165, 165, 170, 170, 135])

        # Inspire hand joint angle limits (scaled 0-1000)
        self.inspire_hand_joint_min = rospy.get_param('inspire_hand_joint_min', [0, 0, 0, 0, 0, 0])
        self.inspire_hand_joint_max = rospy.get_param('inspire_hand_joint_max', [1000, 1000, 1000, 1000, 1000, 1000])

        # Predefined angles for gestures
        self.fist_angles = rospy.get_param('fist_angles', [0, 0, 0, 0, 0, 1000])
        self.pinch_index_angles = rospy.get_param('pinch_index_angles', [-1, -1, -1, 482, 560, 110])

        # Thresholds for gestures
        self.index_pinch_threshold = rospy.get_param('index_pinch_threshold', 0.06)
        self.fist_threshold = rospy.get_param('fist_threshold', 0.19)

        hand_prefix = 'L' if self.hand_side == 'left' else 'R'
        self.joint_names = [f"{hand_prefix}{suffix}" for suffix in JOINT_NAMES_SUFFIXES]
        
        # Subscriber to the Leap Motion data topic
        self.leap_sub = rospy.Subscriber(self.leap_topic, Hand, self.leap_callback, queue_size=1)

        # Publisher for the leap hand joint angles topic
        self.hand_pub = rospy.Publisher(self.hand_ns + '/leap_teleop/hand_angles', Int32MultiArray, queue_size=1)

        self.last_leap_positions = [1000] * len(self.joint_names) # Initialize last leap positions to a high value (fully open)

        self.rate = rospy.Rate(self.freq)  # Set the loop rate

        rospy.loginfo("Inspire Hand Teleop node ready!")



    def leap_callback(self, msg):
        """
        Callback function to process Leap Motion data and update joint angles.
        """
        angles = [0] * len(self.joint_names)  # Initialize angles for each joint
        try:
            fingerlist = msg.finger_list_filtered
            # 1. DIRECT MAPPING from Leap Motion data
            # Reversed loop to process from pinky to thumb, matching the Inspire hand joint order
            for i, finger in list(enumerate(fingerlist)): 
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
                    pinky_first_bone_start = fingerlist[-1].bone_list[0].bone_start
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
            thumb_tip = fingerlist[0].bone_list[-1].bone_end
            index_tip = fingerlist[1].bone_list[-1].bone_end
            middle_tip = fingerlist[2].bone_list[-1].bone_end
            ring_tip = fingerlist[3].bone_list[-1].bone_end
            pinky_tip = fingerlist[4].bone_list[-1].bone_end
            palm_center = msg.palm_center.position

            thumb_to_index_distance = self.euclidean_distance(thumb_tip, index_tip)
            thumb_to_palm_distance = self.euclidean_distance(thumb_tip, palm_center)
            index_to_palm_distance = self.euclidean_distance(index_tip, palm_center)
            middle_to_palm_distance = self.euclidean_distance(middle_tip, palm_center)
            ring_to_palm_distance = self.euclidean_distance(ring_tip, palm_center)
            pinky_to_palm_distance = self.euclidean_distance(pinky_tip, palm_center)
            total_distance_to_palm = (thumb_to_palm_distance + index_to_palm_distance + middle_to_palm_distance + ring_to_palm_distance + pinky_to_palm_distance)

            # Check for FIST
            if total_distance_to_palm < self.fist_threshold:
                # rospy.loginfo_throttle(1, "FIST gesture detected.")
                mapped_angles = self.fist_angles  # Set angles for a fist gesture
            # Check for PINCH with the index finger
            elif thumb_to_index_distance < self.index_pinch_threshold:
                # rospy.loginfo_throttle(1, "Thumb is close to the INDEX finger: " + str(thumb_to_index_distance))
                # Compute blend factor: 0 (no pinch) to 1 (full pinch)
                blend = max(0.0, min(1.0, 1.0 - ((thumb_to_index_distance - 0.005) / self.index_pinch_threshold)))
                # rospy.loginfo("Thumb to index distance: " + str(thumb_to_index_distance) + ", index pinch threshold: " + str(self.index_pinch_threshold) + ", blend factor: " + str(blend))
                for i in range(3, 6):
                    pinch_val = self.pinch_index_angles[i]
                    if pinch_val != -1:
                        mapped_angles[i] = int((1- blend) * mapped_angles[i] + blend * pinch_val)
            # Finally, for SAFETY, make sure that if the index is closed, the thumb is opened, and vice versa, to avoid collisions
            elif mapped_angles[3] < self.pinch_index_angles[3]:
                # rospy.loginfo_throttle(1,"INDEX finger is closed, opening Thumb's 2nd DoF.")
                if mapped_angles[5] < 600:
                    mapped_angles[4] = max(mapped_angles[4], 350)
                    mapped_angles[5] = 600
                else:
                    mapped_angles[4] = max(mapped_angles[4], 200)


            # Finally, update the last joint positions
            self.last_leap_positions = [float(self.smoothing_factor * current_angle + (1 - self.smoothing_factor) * last_angle) for current_angle, last_angle in zip(mapped_angles, self.last_leap_positions)]

        except IndexError:
            rospy.logerr("Leap Motion data does not contain enough values.")


    def run(self):
        while not rospy.is_shutdown():
            if any(position != -1 for position in self.last_leap_positions):
                # Publish the smoothed joint angles
                leap_hand_msg = Int32MultiArray()
                leap_hand_msg.data = [int(pos) for pos in self.last_leap_positions]
                self.hand_pub.publish(leap_hand_msg)

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
        node = LeapToInspireHand()
        node.run()
    except rospy.ROSInterruptException:
        pass