#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

class RoboticArmController:
    def __init__(self):
        rospy.init_node('arm_controller', anonymous=True)
        
        # Correct publisher names to match controller.yaml
        self.base_pub = rospy.Publisher('/FusionComponent/top_base_rotator_joint_position_controller/command', Float64, queue_size=10)
        self.shoulder_pub = rospy.Publisher('/FusionComponent/shoulder_motor_axel_joint_position_controller/command', Float64, queue_size=10)
        self.elbow_pub = rospy.Publisher('/FusionComponent/elbow_joint_position_controller/command', Float64, queue_size=10)
        self.wrist_pub = rospy.Publisher('/FusionComponent/wrist_joint_position_controller/command', Float64, queue_size=10)
        self.palm_pub = rospy.Publisher('/FusionComponent/palm_motor_gear_joint_position_controller/command', Float64, queue_size=10)
        self.left_finger_bottom_pub = rospy.Publisher('/FusionComponent/left_finger_bottom_gear_joint_position_controller/command', Float64, queue_size=10)
        self.right_finger_bottom_pub = rospy.Publisher('/FusionComponent/right_finger_bottom_gear_joint_position_controller/command', Float64, queue_size=10)
        self.left_finger_top_pub = rospy.Publisher('/FusionComponent/left_finger_top_gear_joint_position_controller/command', Float64, queue_size=10)

        # Wait for all publishers to be ready
        rospy.sleep(2)  # Allow time for ROS to register publishers

    def move_joint(self, publisher, position, delay=1.0):
        """ Publishes a position to a given joint and waits for execution """
        if not rospy.is_shutdown():
            publisher.publish(Float64(position))
            rospy.sleep(delay)  # Allow movement to complete

    def open_gripper(self):
        """Opens the gripper"""
        rospy.loginfo("Opening gripper...")
        self.move_joint(self.palm_pub, 0.0)
        self.move_joint(self.left_finger_bottom_pub, 0.0)
        self.move_joint(self.right_finger_bottom_pub, 0.0)
        self.move_joint(self.left_finger_top_pub, 0.0)

    def close_gripper(self):
        """Closes the gripper"""
        rospy.loginfo("Closing gripper...")
        self.move_joint(self.palm_pub, 1.5)
        self.move_joint(self.left_finger_bottom_pub, 1.5)
        self.move_joint(self.right_finger_bottom_pub, 1.5)
        self.move_joint(self.left_finger_top_pub, 1.5)

    def pick_up_sequence(self):
        """ Executes a simple pick-up sequence """
        rospy.loginfo("Starting pick-up sequence...")

        # Move to starting position
        self.move_joint(self.shoulder_pub, 0.0, 2.0)
        self.move_joint(self.elbow_pub, 0.0, 2.0)
        self.move_joint(self.wrist_pub, 0.0, 2.0)

        # Open gripper
        self.open_gripper()

        # Move to object position
        rospy.loginfo("Moving to object...")
        self.move_joint(self.shoulder_pub, -0.5, 2.0)
        self.move_joint(self.elbow_pub, 1.0, 2.0)
        self.move_joint(self.wrist_pub, 0.1, 2.0)

        # Close gripper (Pick up object)
        self.close_gripper()

        # Lift the object
        rospy.loginfo("Lifting object...")
        self.move_joint(self.shoulder_pub, 0.0, 2.0)
        self.move_joint(self.elbow_pub, 0.5, 2.0)

        rospy.loginfo("Pick-up sequence completed.")

if __name__ == '__main__':
    try:
        controller = RoboticArmController()
        controller.pick_up_sequence()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pick-up sequence interrupted.")

