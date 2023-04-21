#!/usr/bin/env python3

# # Author: Benjamin Dowden
## creates a publisher for the current box location and up vector

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from std_msgs.msg import String
import numpy as np
import tf

# Initialize moveit_commander and rospy
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('my_publisher_node')
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# declare update rate of 2 Hz or 500 ms
rate = rospy.Rate(2)

# Create a MoveGroupCommander object for the arm
arm_group_name = "panda_arm"
arm_group = moveit_commander.MoveGroupCommander(arm_group_name)
eef_link = arm_group.get_end_effector_link()
scene = moveit_commander.PlanningSceneInterface()

# Create a publisher for the box pose
box_position_pub = rospy.Publisher('/box_position', String, queue_size=10)
box_orientation_pub = rospy.Publisher('/box_orientation', String, queue_size=10)

# Define the name of the box
box_name = "box"

# Retrieve the current pose of the box
while not rospy.is_shutdown():
    try:

        # get the transform of the base to eef for pose updates
        object_transform = tf_buffer.lookup_transform('panda_link0', eef_link, rospy.Time(0), rospy.Duration(1.0))

        # Obtain information about the box pose & attached state
        box_pose = scene.get_objects()
        attached_objects = scene.get_attached_objects()

        # Update information if the box is not attached to the gripper
        if box_pose:
            # convert the orientation data to the 'up' vector
            quaternion = [box_pose['box'].pose.orientation.x, box_pose['box'].pose.orientation.y, box_pose['box'].pose.orientation.z,box_pose['box'].pose.orientation.w]
            rotation_matrix = tf.transformations.quaternion_matrix(quaternion)
            up_vector = rotation_matrix[:3, 1]
            up_vector /= np.linalg.norm(up_vector)

            # populate the publisher messages
            position_msg = f"x: {box_pose['box'].pose.position.x:.3f}, y: {box_pose['box'].pose.position.y:.3f}, z:{box_pose['box'].pose.position.z:.3f}"
            orientation_msg = f"[{up_vector[0]:.3f},{up_vector[1]:.3f},{up_vector[2]:.3f}]"

        # else condition if the box is attached as it will not exist as a scene object
        elif attached_objects:
            # convert the orientation data to the 'up' vector
            quaternion = [object_transform.transform.rotation.x, object_transform.transform.rotation.y, object_transform.transform.rotation.z,object_transform.transform.rotation.w]
            rotation_matrix = tf.transformations.quaternion_matrix(quaternion)
            up_vector = rotation_matrix[:3, 1]
            up_vector /= np.linalg.norm(up_vector)

            # populate the publisher messages
            position_msg = f"x: {object_transform.transform.translation.x:.3f}, y: {object_transform.transform.translation.y:.3f}, z:{object_transform.transform.translation.z:.3f}"
            orientation_msg = f"[{up_vector[0]:.3f},{up_vector[1]:.3f},{up_vector[2]:.3f}]"

        # send update if the box object is non existent
        else:
            position_msg = "Are you sure the box is spawned?"
            orientation_msg = "Are you sure the box is spawned?"

                
        # Publish the box pose
        box_position_pub.publish(position_msg)
        box_orientation_pub.publish(orientation_msg)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn('Failed to get transform: %s', e)
    rate.sleep()