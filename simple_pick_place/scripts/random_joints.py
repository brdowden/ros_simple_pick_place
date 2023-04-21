#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from std_msgs.msg import String
import random
import tf
from math import pi

tau = 2.0 * pi
class RandomJointMovement(object):
    def __init__(self):
        super(RandomJointMovement, self).__init__()
                #initialize robot
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("random_joint_state", anonymous=True)

                ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
                ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

                ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
                ## for getting, setting, and updating the robot's internal understanding of the
                ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

                ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
                ## to a planning group (group of joints).  
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        self.move_group = move_group
        self.robot = robot
        self.scene = scene

    def go_to_joints(self):
            """ 
            Defines a random joint target to move the robot to between typical safe bounds for the robot.
            Please note that this does not account for singularities.
            
            """
            move_group = self.move_group

            # decleare random pose goals
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = random.uniform(-tau/4, tau/4)
            joint_goal[1] = random.uniform(-tau/4, tau/4)
            joint_goal[2] = random.uniform(0, tau/4)
            joint_goal[3] = random.uniform(-tau/4, tau/4)
            joint_goal[4] = random.uniform(-tau/4, tau/4)
            joint_goal[5] = random.uniform(0, tau/4)
            joint_goal[6] = random.uniform(-tau/4, tau/4)

            # execute a joint move to the position
            success = move_group.go(joint_goal, wait=True)
            

            # Calling ``stop()`` ensures that there is no residual movement
            move_group.stop()
            move_group.clear_pose_targets()




def main():
    try:
        movement = RandomJointMovement()
        movement.go_to_joints()
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
