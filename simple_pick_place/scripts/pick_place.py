#!/usr/bin/env python


# # Author: Benjamin Dowden

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from moveit_commander.conversions import pose_to_list



class PickPlaceInterface(object):
    """PickPlaceInterface"""

    def __init__(self):
        super(PickPlaceInterface, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object.
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate necessary `MoveGroupCommander`_ objects.
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        group_name = "panda_hand"    
        gripper_group = moveit_commander.MoveGroupCommander(group_name)

        group_name = "panda_manipulator"    
        pick_group = moveit_commander.MoveGroupCommander(group_name)
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        
    
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # Get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # Print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # List of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.gripper_group = gripper_group
        self.pick_group = pick_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


    def wait_for_state_update(
            self, box_is_known=False, box_is_attached=False, timeout=4
        ):
            """
            Wait for the planning scene to update based on changes to object or attachment state.

            Args:
                box_is_known (bool): Whether the box is known in the planning scene (default: False).
                box_is_attached (bool): Whether the box is attached to the robot (default: False).
                timeout (float): Timeout in seconds to wait for the planning scene update (default: 4).

            Returns:
                bool: True if the planning scene update was successful, False otherwise.
            """
            box_name = self.box_name
            scene = self.scene

            start = rospy.get_time()
            seconds = rospy.get_time()
            while (seconds - start < timeout) and not rospy.is_shutdown():
                # Test if the box is in attached objects
                attached_objects = scene.get_attached_objects([box_name])
                is_attached = len(attached_objects.keys()) > 0

                # Test if the box is in the scene.
                # Note that attaching the box will remove it from known_objects
                is_known = box_name in scene.get_known_object_names()

                # Test if we are in the expected state
                if (box_is_attached == is_attached) and (box_is_known == is_known):
                    return True

                # Sleep so that we give other threads time on the processor
                rospy.sleep(0.1)
                seconds = rospy.get_time()

            # If we exited the while loop without returning then we timed out
            return False
    

    def add_box(self, timeout=4):
        """
        Spawns a box in the rviz environment

        Args:
            timeout (float): Timeout in seconds to wait for the planning scene update (default: 4).
        """

        box_name = self.box_name
        scene = self.scene

        ## Create a box in the planning scene at a set spawn point:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_link0" # use the world frame
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.7 
        box_pose.pose.position.y = 0  
        box_pose.pose.position.z = 0.05/2
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))
        
        # Create the publisher node for the spawn point position. It is assumed the box is on the ground.
        self.publish_message('/spawn_point',f'The box starting position is: x= {box_pose.pose.position.x}, y= {box_pose.pose.position.y}, z= {0}')

        # update box variable & wait for load confirmation
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    
    def attach_box(self, timeout=4):
        """
        Attachs the box object to the robot gripper

        Args:
            timeout (float): Timeout in seconds to wait for the planning scene update (default: 4).
        """

        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link

        # Define grasping group function & attach box to the eef
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )


    def detach_box(self, timeout=4):
        """
        Detachs the box object to the robot gripper

        Args:
            timeout (float): Timeout in seconds to wait for the planning scene update (default: 4).
        """

        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## Detach the box Object from the Robot
        scene.remove_attached_object(eef_link, name=box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    
    def go_to_pose(self, x, y, z, roll, pitch, yaw):
        
        """
        Moves the robot to a declared pose

        Args:
            x (float): x coordinate of the desired pose, given in metres.
            y (float): y coordinate of the desired pose, given in metres.
            z (float): z coordinate of the desired pose, given in metres.
            roll (float): roll angle of the desired pose, given in radians.
            pitch (float): pitch angle of the desired pose, given in radians.
            yaw (float): yaw angle of the desired pose, given in radians.
        """

        move_group = self.move_group

        # convert rpy into quaternion
        q = quaternion_from_euler(roll,pitch,yaw)

        # define an empty pose goal for the robot
        pose_goal = geometry_msgs.msg.Pose()
        
        # populate the pose goal
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.w = q[3]
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        
        # Set the pose target for the robot and move to the pose
        move_group.set_pose_target(pose_goal)    
        success = move_group.go(wait=True)

        # stop the robot & clear pose goal
        move_group.stop()
        move_group.clear_pose_targets()


    def go_to_joints(self, j0, j1, j2, j3, j4, j5, j6):
        """
        Moves the robot to a declared joint goal.

        Args:
            j0 (float): desired joint 1 state, given in metres.
            j1 (float): desired joint 2 state, given in metres.
            j2 (float): desired joint 3 state, given in metres.
            j3 (float): desired joint 4 state, given in radians.
            j4 (float): desired joint 5 state, given in radians.
            j5 (float): desired joint 6 state, given in radians.
            j6 (loat): desired joint 7 state, given in radians.

        """

        move_group = self.move_group

        # Get current joint values & replace with desired new joint values
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = j0
        joint_goal[1] = j1
        joint_goal[2] = j2
        joint_goal[3] = j3
        joint_goal[4] = j4
        joint_goal[5] = j5  
        joint_goal[6] = j6

        # Set the joint target for the robot and move to the pose
        success = move_group.go(joint_goal, wait=True)
        
        # Stop the robot & clear pose goal
        move_group.stop()
        move_group.clear_pose_targets()


    def open_gripper(self):
        """
            Opens the panda end effector gripper
        """
        
        # Get decared gripper group and go to set target 'open'
        gripper_group = self.gripper_group
        gripper_group.set_named_target('open')
        success = gripper_group.go(wait=True)
    

    def close_gripper(self):
        """
            Closes the panda end effector gripper
        """

        # Get decared gripper group and go to set target 'close'
        gripper_group = self.gripper_group   
        gripper_group.set_named_target('close')
        success = gripper_group.go(wait=True)
   

    def cartesian_move(self, x,y,z):
        """
            Defines a cartesian move goal for the robot to follow.

        Args:
            x (float): desired x position movement, in metres.
            y (float): desired y position movement, in metres.
            z (float): desired z position movement, in metres.
        """

        move_group = self.move_group
        waypoints = []

        # Get current pose data and add desired position change to it
        wpose = move_group.get_current_pose().pose
        wpose.position.x += x 
        wpose.position.y += y
        wpose.position.z += z  
        waypoints.append(copy.deepcopy(wpose))

        # Use a cartesian planner to compute the move path
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        move_group.execute(plan,wait=True)
        return


    def publish_message(self,node_name,info):
        """
            Creates a publisher node for a single instance publish.
            NOTE: only uses String msgs

        Args:
            node_name (string): The desired publisher node name.
            info (string): The desired string published information.
        """
        
        # create publisher node and rest to allow generation
        pub = rospy.Publisher(node_name, String, queue_size=1)
        rate = rospy.Rate(2)
        rate.sleep()
    
        # publish msg to the topic
        msg = String()
        msg.data = info
        rospy.loginfo(msg)
        pub.publish(msg)
        



def main():
    try:
        # load functions and start ros node
        pick_place = PickPlaceInterface()

        # publish drop zone information to the ros topic
        pick_place.publish_message('/drop_zone',"The drop zone is: x= 0, y= 0.7, z= 0")

        #spawn box & send robot to a starting position
        pick_place.add_box()
        pick_place.go_to_joints(0,-tau/8,0,-tau/4,0,tau/6,0)

        # open gripper 
        pick_place.open_gripper()

        # approach box location & cartesian move to the target for the last 0.1m
        pick_place.go_to_pose(0.6,0.0,0.025,-tau / 4, -tau / 8, -tau / 4)
        pick_place.cartesian_move(0.1,0,0)

        # close the gripper & pick up the box
        pick_place.close_gripper()
        pick_place.attach_box()

        # lift robot arm & go to drop zone pose approach
        pick_place.cartesian_move(0,0,0.2)
        pick_place.go_to_pose(0,0.6,0.2,-tau /4, -5*tau / 8, 0)
        pick_place.cartesian_move(0,0,-0.10)
        
        # Open gripper & drop the box at the drop zone
        pick_place.open_gripper()
        pick_place.detach_box()
        pick_place.cartesian_move(0,-0.2,0)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/noetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
