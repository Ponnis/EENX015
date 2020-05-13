#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import numpy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros as tf2
import tf2_geometry_msgs
import tf
from math import pi, sqrt, atan2
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Twist
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
import tf2_ros
import traceback
import time

# Global variables:
aruco_cb = geometry_msgs.msg.PoseStamped()
list = [[],[],[],[],[],[],[]]
found = False

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "iiwa"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.base_name = 'base'
    self.w0_name = 'wheel0'
    self.w1_name = 'wheel1'
    self.top_name = 'top'
    self.screw_name = 'screw'
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.base_pose = geometry_msgs.msg.PoseStamped()
    self.w0_pose = geometry_msgs.msg.PoseStamped()
    self.w1_pose = geometry_msgs.msg.PoseStamped()
    self.top_pose = geometry_msgs.msg.PoseStamped()
    self.screw_pose = geometry_msgs.msg.PoseStamped()
    self.aruco_pose = geometry_msgs.msg.PoseStamped()
    self.aruco_res = geometry_msgs.msg.PoseStamped()
    self.base_pos = [0, 0.11, 0.01]
    self.w0_pos = [-0.1, 0.12, 0.005]
    self.w1_pos = [-0.08, 0.12, 0.005]
    self.top_pos = [0, 0.2, 0.013]
    self.screw_pos = [-0.1, 0.2, 0.025]


  def get_object_pose(self, obj):
      if (obj == 'base'):
          o_pose = self.base_pose
      elif (obj == 'wheel0'):
          o_pose = self.w0_pose
      elif (obj == 'wheel1'):
          o_pose = self.w1_pose
      elif (obj == 'top'):
          o_pose = self.top_pose

      return (o_pose)

  def median(self,l):
        return numpy.median(numpy.array(l))

  def go_up(self):

      move_group = self.move_group

      pose_goal = geometry_msgs.msg.Pose()
      current_pose = self.move_group.get_current_pose().pose
      pose_goal.orientation = current_pose.orientation
      pose_goal.position = current_pose.position
      pose_goal.position.z = current_pose.position.y + 0.1

      # pose_goal.orientation.x = 1
      # pose_goal.orientation.y = 0
      # pose_goal.orientation.z = 0
      # pose_goal.orientation.w = 0
      # pose_goal.position.x = o_pose.pose.position.x
      # pose_goal.position.y = o_pose.pose.position.y - 0.05
      # pose_goal.position.z = o_pose.pose.position.z + 0.02
      print "got here"
      move_group.set_pose_target(pose_goal)
      print "set pose"
      ## Now, we call the planner to compute the plan and execute it.
      plan = move_group.go(wait=True)
      print "moved"
      # Calling `stop()` ensures that there is no residual movement
      move_group.stop()
      # It is always good to clear your targets after planning with poses.
      # Note: there is no equivalent function for clear_joint_value_targets()
      move_group.clear_pose_targets()

      ## END_SUB_TUTORIAL

      # For testing:
      # Note that since this section of code will not be included in the tutorials
      # we use the class variable rather than the copied state variable
      current_pose = self.move_group.get_current_pose().pose
      return all_close(pose_goal, current_pose, 1000)

  def go_to_object_pose(self, obj):
      o_pose = geometry_msgs.msg.PoseStamped()

      if (obj == 'wheel0'):
          o_pose = self.w0_pose
      elif (obj == 'wheel1'):
          o_pose = self.w1_pose
      elif (obj == 'top'):
          o_pose = self.top_pose

      if (obj == 'base'):
          o_pose.pose.position.x = 0
          o_pose.pose.position.y = 0
          o_pose.pose.position.z = 0


      move_group = self.move_group
      print(str(o_pose))

      pose_goal = geometry_msgs.msg.Pose()
      pose_goal.orientation.x = 1
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      pose_goal.orientation.w = 0
      pose_goal.position.x = self.base_pose.pose.position.x + o_pose.pose.position.x
      pose_goal.position.y = self.base_pose.pose.position.y + o_pose.pose.position.y - 0.05
      pose_goal.position.z = self.base_pose.pose.position.z + o_pose.pose.position.y +0.05
      print "got here"
      move_group.set_pose_target(pose_goal)

      ## Now, we call the planner to compute the plan and execute it.
      plan = move_group.go(wait=True)
      # Calling `stop()` ensures that there is no residual movement
      move_group.stop()
      # It is always good to clear your targets after planning with poses.
      # Note: there is no equivalent function for clear_joint_value_targets()
      move_group.clear_pose_targets()

      ## END_SUB_TUTORIAL

      # For testing:
      # Note that since this section of code will not be included in the tutorials
      # we use the class variable rather than the copied state variable
      current_pose = self.move_group.get_current_pose().pose
      return all_close(pose_goal, current_pose, 1000)


  def get_joint_states(self):
      move_group = self.move_group
      state = move_group.get_current_joint_values()
      return state

  def go_to_joint_state(self, arr):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = arr[0]
    joint_goal[1] = arr[1]
    joint_goal[2] = arr[2]
    joint_goal[3] = arr[3]
    joint_goal[4] = arr[4]
    joint_goal[5] = arr[5]
    joint_goal[6] = arr[6]

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 1)


  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group
    base_pose = self.base_pose
    aruco_res = self.aruco_res
    current_pose = self.move_group.get_current_pose().pose
    #print(current_pose)
    #print(pose_to_list(current_pose))
    #print(self.move_group.get_current_joint_values())
    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    q = quaternion_from_euler(0, 0, 0)
    pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0
    # pose_goal.orientation.x = 1
    # pose_goal.orientation.y = 0
    # pose_goal.orientation.z = 0
    # pose_goal.position.x = 0.30
    # pose_goal.position.y = 0.85 #pose - 0.05
    # pose_goal.position.z = 0.96 #pose + 0.02
    print(base_pose.pose)
    pose_goal.orientation.w = 0
    pose_goal.orientation.x = 1
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0
    pose_goal.position.x = aruco_res.pose.position.x
    pose_goal.position.y = aruco_res.pose.position.y - 0.05
    pose_goal.position.z = aruco_res.pose.position.z + 0.02

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 1000)

  def go_to_base(self):

      move_group = self.move_group
      base_pose = self.base_pose
      aruco_res = self.aruco_res

      q = quaternion_from_euler(0, 0, 0)
      pose_goal = geometry_msgs.msg.Pose()

      print(base_pose.pose)
      pose_goal.orientation.w = 0
      pose_goal.orientation.x = 1
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      pose_goal.position.x = aruco_res.pose.position.x + base_pose.pose.position.y
      pose_goal.position.y = aruco_res.pose.position.y + base_pose.pose.position.x - 0.05
      pose_goal.position.z = aruco_res.pose.position.z + base_pose.pose.position.z + 0.02

      move_group.set_pose_target(pose_goal)
      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()
      current_pose = self.move_group.get_current_pose().pose
      return all_close(pose_goal, current_pose, 1000)

  def go_to_w0(self):

      move_group = self.move_group
      base_pose = self.base_pose
      w0_pose = self.w0_pose
      aruco_res = self.aruco_res

      q = quaternion_from_euler(0, 0, 0)
      pose_goal = geometry_msgs.msg.Pose()

      print(w0_pose)
      pose_goal.orientation.w = 0
      pose_goal.orientation.x = 1
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      pose_goal.position.x = aruco_res.pose.position.x + self.w0_pos[0]
      pose_goal.position.y = aruco_res.pose.position.y + self.w0_pos[1] - 0.046
      pose_goal.position.z = aruco_res.pose.position.z + 0.02

      move_group.set_pose_target(pose_goal)
      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()
      current_pose = self.move_group.get_current_pose().pose
      return all_close(pose_goal, current_pose, 1000)

  def go_to_w1(self):

      move_group = self.move_group
      base_pose = self.base_pose
      w0_pose = self.w0_pose
      aruco_res = self.aruco_res

      q = quaternion_from_euler(0, 0, 0)
      pose_goal = geometry_msgs.msg.Pose()

      print(w0_pose)
      pose_goal.orientation.w = 0
      pose_goal.orientation.x = 1
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      pose_goal.position.x = aruco_res.pose.position.x + self.w1_pos[0]
      pose_goal.position.y = aruco_res.pose.position.y + self.w1_pos[1] - 0.045
      pose_goal.position.z = aruco_res.pose.position.z + 0.02

      move_group.set_pose_target(pose_goal)
      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()
      current_pose = self.move_group.get_current_pose().pose
      return all_close(pose_goal, current_pose, 1000)

  def go_to_top(self):

      move_group = self.move_group
      base_pose = self.base_pose
      aruco_res = self.aruco_res

      q = quaternion_from_euler(0, 0, 0)
      pose_goal = geometry_msgs.msg.Pose()

      print(self.top_pose)
      pose_goal.orientation.w = 0
      pose_goal.orientation.x = 1
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      pose_goal.position.x = aruco_res.pose.position.x + self.top_pos[0]
      pose_goal.position.y = aruco_res.pose.position.y + self.top_pos[1] - 0.045
      pose_goal.position.z = aruco_res.pose.position.z + 0.02

      move_group.set_pose_target(pose_goal)
      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()
      current_pose = self.move_group.get_current_pose().pose
      return all_close(pose_goal, current_pose, 1000)

  def go_to_screw(self):

      move_group = self.move_group
      base_pose = self.base_pose
      screw_pose = self.screw_pose
      aruco_res = self.aruco_res

      q = quaternion_from_euler(0, 0, 0)
      pose_goal = geometry_msgs.msg.Pose()

      pose_goal.orientation.w = 0
      pose_goal.orientation.x = 1
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      pose_goal.position.x = aruco_res.pose.position.x + self.screw_pos[0]
      pose_goal.position.y = aruco_res.pose.position.y + self.screw_pos[1] - 0.05
      pose_goal.position.z = aruco_res.pose.position.z + 0.04

      move_group.set_pose_target(pose_goal)
      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()
      current_pose = self.move_group.get_current_pose().pose
      return all_close(pose_goal, current_pose, 1000)

  def go_to_screw_detach(self):

      move_group = self.move_group
      base_pose = self.base_pose
      w0_pose = self.w0_pose
      aruco_res = self.aruco_res

      q = quaternion_from_euler(0, 0, 0)
      pose_goal = geometry_msgs.msg.Pose()

      pose_goal.orientation.w = 0
      pose_goal.orientation.x = 1
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      pose_goal.position.x = aruco_res.pose.position.x + self.base_pos[0]
      pose_goal.position.y = aruco_res.pose.position.y + self.base_pos[1] - 0.05
      pose_goal.position.z = aruco_res.pose.position.z + self.base_pos[2] + 0.05

      move_group.set_pose_target(pose_goal)
      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()

      # pose_goal.position.z = aruco_res.pose.position.z + base_pose.pose.position.z + 0.02
      # move_group.set_pose_target(pose_goal)
      # plan = move_group.go(wait=True)
      # move_group.stop()
      # move_group.clear_pose_targets()

      current_pose = self.move_group.get_current_pose().pose
      return all_close(pose_goal, current_pose, 1000)

  def go_to_top_detach(self):

      move_group = self.move_group
      base_pose = self.base_pose
      w0_pose = self.w0_pose
      aruco_res = self.aruco_res

      q = quaternion_from_euler(0, 0, 0)
      pose_goal = geometry_msgs.msg.Pose()

      pose_goal.orientation.w = 0
      pose_goal.orientation.x = 1
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      pose_goal.position.x = aruco_res.pose.position.x + self.base_pos[0]
      pose_goal.position.y = aruco_res.pose.position.y + self.base_pos[1] - 0.045
      pose_goal.position.z = aruco_res.pose.position.z + self.base_pos[2] + 0.05
      move_group.set_pose_target(pose_goal)
      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()

      pose_goal.position.z = aruco_res.pose.position.z + self.base_pos[2] + 0.02
      move_group.set_pose_target(pose_goal)
      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()

      current_pose = self.move_group.get_current_pose().pose
      return all_close(pose_goal, current_pose, 1000)


  def go_to_w0_detach(self):

      move_group = self.move_group
      base_pose = self.base_pose
      w0_pose = self.w0_pose
      aruco_res = self.aruco_res

      q = quaternion_from_euler(0, 0, 0)
      pose_goal = geometry_msgs.msg.Pose()

      pose_goal.orientation.w = 0
      pose_goal.orientation.x = 1
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      pose_goal.position.x = aruco_res.pose.position.x + self.base_pos[0] + 0.02
      pose_goal.position.y = aruco_res.pose.position.y + self.base_pos[1] - 0.045
      pose_goal.position.z = aruco_res.pose.position.z + self.base_pos[2] + 0.05

      move_group.set_pose_target(pose_goal)
      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()

      pose_goal.position.z = aruco_res.pose.position.z + base_pose.pose.position.z + 0.02
      move_group.set_pose_target(pose_goal)
      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()

      current_pose = self.move_group.get_current_pose().pose
      return all_close(pose_goal, current_pose, 1000)

  def go_to_w1_detach(self):

      move_group = self.move_group
      base_pose = self.base_pose
      w0_pose = self.w0_pose
      aruco_res = self.aruco_res

      q = quaternion_from_euler(0, 0, 0)
      pose_goal = geometry_msgs.msg.Pose()

      pose_goal.orientation.w = 0
      pose_goal.orientation.x = 1
      pose_goal.orientation.y = 0
      pose_goal.orientation.z = 0
      pose_goal.position.x = aruco_res.pose.position.x + self.base_pos[0] - 0.02
      pose_goal.position.y = aruco_res.pose.position.y + self.base_pos[1] - 0.045
      pose_goal.position.z = aruco_res.pose.position.z + self.base_pos[2] + 0.05
      move_group.set_pose_target(pose_goal)
      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()

      pose_goal.position.z = aruco_res.pose.position.z + base_pose.pose.position.z + 0.02
      move_group.set_pose_target(pose_goal)
      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()

      current_pose = self.move_group.get_current_pose().pose
      return all_close(pose_goal, current_pose, 1000)

  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []


    wpose = move_group.get_current_pose().pose

    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwardmove_groups in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL


  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    base_name = self.base_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([base_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = base_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def add_marker(self, timeout=4):

    name = "marker"
    self.scene.add_box(name, self.aruco_res, size=(0.05,0.014,0.01))#size=(0.014, 0.014, 0.02))
    print "marker added"
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def add_marker_live(self, timeout=4):

    name = "marker"
    self.scene.add_box(name, aruco_cb, size=(0.014,0.014,0.01))#size=(0.014, 0.014, 0.02))
    print "marker added"
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def add_base(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    base_pose = self.base_pose
    w0_pose = self.w0_pose
    w1_pose = self.w1_pose
    top_pose = self.top_pose
    screw_pose = self.screw_pose
    base_name = self.base_name
    w0_name = self.w0_name
    w1_name = self.w1_name
    top_name = self.top_name
    screw_name = self.screw_name
    scene = self.scene

    q = quaternion_from_euler(0, 0, pi/2)
    #base_pose = geometry_msgs.msg.PoseStamped()
    base_pose.header.frame_id = "marker"
    base_pose.pose.orientation.x = q[0]
    base_pose.pose.orientation.y = q[1]
    base_pose.pose.orientation.z = q[2]
    base_pose.pose.orientation.w = q[3]
    base_pose.pose.position.x = self.base_pos[0] #0.16 irl
    base_pose.pose.position.y = self.base_pos[1]
    base_pose.pose.position.z = self.base_pos[2]

    base_name = "base"
    scene.add_box(base_name, base_pose, size=(0.045, 0.087, 0.02))

    #Add wheel0
    q = quaternion_from_euler(0, pi/2, pi/2)
    #w0_pose = geometry_msgs.msg.PoseStamped()
    w0_pose.header.frame_id = "marker"
    w0_pose.pose.orientation.x = q[0]
    w0_pose.pose.orientation.y = q[1]
    w0_pose.pose.orientation.z = q[2]
    w0_pose.pose.orientation.w = q[3]
    w0_pose.pose.position.x = self.w0_pos[0] #0.04
    w0_pose.pose.position.y = self.w0_pos[1] #-0.08
    w0_pose.pose.position.z = self.w0_pos[2] #0.005

    w0_name = "wheel0"
    scene.add_cylinder(w0_name, w0_pose, radius=0.006, height=0.065)

    #Add wheel1
    q = quaternion_from_euler(0, pi/2, pi/2)
    #w1_pose = geometry_msgs.msg.PoseStamped()
    w1_pose.header.frame_id = "marker"
    w1_pose.pose.orientation.x = q[0]
    w1_pose.pose.orientation.y = q[1]
    w1_pose.pose.orientation.z = q[2]
    w1_pose.pose.orientation.w = q[3]
    w1_pose.pose.position.x = self.w1_pos[0]
    w1_pose.pose.position.y = self.w1_pos[1]
    w1_pose.pose.position.z = self.w1_pos[2]

    w1_name = "wheel1"
    scene.add_cylinder(w1_name, w1_pose, radius=0.006, height=0.065)

    q = quaternion_from_euler(0, 0, pi/2)
    #top_pose = geometry_msgs.msg.PoseStamped()
    top_pose.header.frame_id = "marker"
    top_pose.pose.orientation.x = q[0]
    top_pose.pose.orientation.y = q[1]
    top_pose.pose.orientation.z = q[2]
    top_pose.pose.orientation.w = q[3]
    top_pose.pose.position.x = self.top_pos[0]
    top_pose.pose.position.y = self.top_pos[1]
    top_pose.pose.position.z = self.top_pos[2]

    top_name = "top"
    scene.add_box(top_name, top_pose, size=(0.045, 0.087, 0.026))

    #Add wheel0
    q = quaternion_from_euler(0, 0, pi/2)
    #w0_pose = geometry_msgs.msg.PoseStamped()
    screw_pose.header.frame_id = "marker"
    screw_pose.pose.orientation.x = q[0]
    screw_pose.pose.orientation.y = q[1]
    screw_pose.pose.orientation.z = q[2]
    screw_pose.pose.orientation.w = q[3]
    screw_pose.pose.position.x = self.screw_pos[0] #0.04
    screw_pose.pose.position.y = self.screw_pos[1] #-0.08
    screw_pose.pose.position.z = self.screw_pos[2] #0.005

    screw_name = "screw"
    scene.add_cylinder(screw_name, screw_pose, radius=0.006, height=0.05)

    self.base_name=base_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_object(self, obj, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.

    if (obj == 'base'):
        to_attach = self.base_name
    elif (obj == 'wheel0'):
        to_attach = self.w0_name
    elif (obj == 'wheel1'):
        to_attach = self.w1_name
    elif (obj == 'top'):
        to_attach = self.top_name
    elif (obj == 'screw'):
        to_attach = self.screw_name

    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names


    grasping_group = 'gripper'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, to_attach, touch_links=touch_links)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_object(self, obj, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    if (obj == 'base'):
        to_detach = self.base_name
    elif (obj == 'wheel0'):
        to_detach = self.w0_name
    elif (obj == 'wheel1'):
        to_detach = self.w1_name
    elif (obj == 'top'):
        to_detach = self.top_name
    elif (obj == 'screw'):
        to_detach = self.screw_name

    scene = self.scene
    eef_link = self.eef_link

    scene.remove_attached_object(eef_link, name=to_detach)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_objects(self, timeout=4):

    scene = self.scene

    scene.remove_world_object(self.base_name)
    scene.remove_world_object(self.w0_name)
    scene.remove_world_object(self.w1_name)
    scene.remove_world_object(self.top_name)
    scene.remove_world_object(self.screw_name)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

def callback(msg):
        imageTime = msg.header.stamp
        # For every fiducial found by the dectector, publish a transform
        for m in msg.transforms:
            id = m.fiducial_id
            trans = m.transform.translation
            rot = m.transform.rotation

            aruco_cb.header.frame_id = "tool_link_ee"
            aruco_cb.pose.orientation.x = 1 #rot.x
            aruco_cb.pose.orientation.y = 0 #rot.y
            aruco_cb.pose.orientation.z = 0 #rot.z
            aruco_cb.pose.orientation.w = 0 #rot.w
            aruco_cb.pose.position.x = trans.x
            aruco_cb.pose.position.y = trans.y
            aruco_cb.pose.position.z = trans.z

            list[0].append(trans.x)
            list[1].append(trans.y)
            list[2].append(trans.z)
            list[3].append(rot.x)
            list[4].append(rot.y)
            list[5].append(rot.z)
            list[6].append(rot.w)

            found = True

def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Pick and Place, adapted from the Moveit Tutorial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Starting ..."
    tutorial = MoveGroupPythonIntefaceTutorial()

    # TRANSFORM LISTENER FOR ARUCO PLACEMENT
    start = [-0.41415453925863344, 0.3715584449139969, 0.7361325368581666, -1.17314415671765, -0.2620980800028647, 1.6138702165508845, 1.7717436871266141]
    start1 = [-2.0834039414342937, -0.6766622038076456, -1.9461502955504797, 1.1706543877977795, -2.3508053969552263, 2.053198583117345, 0.9989990642125767, 0.0, 0.0]
    start2 = [-2.2619144388837618, -0.6842947961826499, -1.7804109003559019, 1.0170783702821486, 0.8496650848667218, -2.0557343545221545, -2.0685291190378554, 0.0004475501727254161, -0.0004475501727254161]
    start3 = [-1.14158059591, 0.672824882423, -0.885950393325, 1.41331959244, 2.48458651934, 2.05816060307, 2.38714031805, 0.000334007592543, -0.000334007592543]
    tutorial.go_to_joint_state(start3)

    #TF
    listener = tf.TransformListener()
    listener.waitForTransform("world", "tool_link_ee", rospy.Time(), rospy.Duration(4.0))
    (trans0, rot0) = listener.lookupTransform('world', 'tool_link_ee', rospy.Time(0))

    #TF2
    tfBuffer = tf2.Buffer()
    list = tf2.TransformListener(tfBuffer)
    tobj = tfBuffer.lookup_transform('world', 'tool_link_ee', rospy.Time(0), rospy.Duration(4.0))

    #From earlier measurement, position of marker relative to camera
    #camera -> marker
    trans1 = [0.0135586191948, -0.0600579480895, 0.61578447076]
    rot1 = [0.999398125078, -0.00114191325631, 0.0196456334891, 0.0285680365118]


    cam_to_aruco = tf2_geometry_msgs.PoseStamped()
    cam_to_aruco.header.frame_id = "tool_link_ee"
    cam_to_aruco.pose.orientation.x = rot1[0]
    cam_to_aruco.pose.orientation.y = rot1[1]
    cam_to_aruco.pose.orientation.z = rot1[2]
    cam_to_aruco.pose.orientation.w = rot1[3]
    cam_to_aruco.pose.position.x = trans1[0]
    cam_to_aruco.pose.position.y = trans1[1]
    cam_to_aruco.pose.position.z = trans1[2]

    aruco_pose_transformed = tf2_geometry_msgs.do_transform_pose(cam_to_aruco, tobj)
    print("From world to marker: ")
    print(str(aruco_pose_transformed))

    atran = aruco_pose_transformed.pose.position
    arot = aruco_pose_transformed.pose.orientation
    print("New pose:" + str(atran) + str(arot))

    #Get the global pose object
    aruco_res = tutorial.aruco_res

    #Some hardcoded values since we know what they are "supposed" to be
    #To use the captures values, use the values in the comments
    aruco_res.header.frame_id = "world"
    aruco_res.pose.orientation.x = 0 #arot.x
    aruco_res.pose.orientation.y = 0 #arot.y
    aruco_res.pose.orientation.z = 0 #arot.z
    aruco_res.pose.orientation.w = 1 #arot.w
    aruco_res.pose.position.x = atran.x
    aruco_res.pose.position.y = atran.y
    aruco_res.pose.position.z = 0.93 #atran.z

    raw_input("Press `Enter` to add marker")
    tutorial.add_marker()
    print(str(aruco_res))

    print "============ Press `Enter` to move the robot ..."
    tutorial.remove_objects()
    raw_input()
    tutorial.add_base()
    start = [-0.41415453925863344, 0.3715584449139969, 0.7361325368581666, -1.17314415671765, -0.2620980800028647, 1.6138702165508845, 1.7717436871266141]
    start1 = [-2.0834039414342937, -0.6766622038076456, -1.9461502955504797, 1.1706543877977795, -2.3508053969552263, 2.053198583117345, 0.9989990642125767, 0.0, 0.0]
    start2 = [-2.2619144388837618, -0.6842947961826499, -1.7804109003559019, 1.0170783702821486, 0.8496650848667218, -2.0557343545221545, -2.0685291190378554, 0.0004475501727254161, -0.0004475501727254161]

    #tutorial.go_to_pose_goal() #go to marker
    # tutorial.go_to_joint_state(start2)


    # print "============ Going to first object "
    # first = [0.5472679466193449, 1.169132990300362, -0.9911353951578363, -1.3482463949438375, 0.985989433873197, 1.1751899751206594, -0.5343455639834627]
    # tutorial.go_to_base()
    # #tutorial.go_to_object_pose('base')
    # print("went to base")
    # tutorial.go_to_pose_goal()

    print "============ Press `Enter` to go to wheel0 object"
    raw_input()
    wheel0_pos = [0.8756192498248672, 1.4651931415513568, -1.4451235064740584, -1.4011732602564302, 1.4101140508232206, 1.4483739241363474, -0.49654546305974134]
    # tutorial.go_to_joint_state(wheel0_pos)
    tutorial.go_to_w0()
    # tutorial.go_to_pose_goal()

    print "============ Attaching object ..."
    obj = 'wheel0'
    tutorial.attach_object('wheel0', 4)
    print "Object attached"

    print "============ Press `Enter` to go to placement position"
    raw_input()
    detach_wheel0 = [-2.928892028186479, -0.8667535063312142, 2.766639956107351, -1.4389499482037884, 0.36402599303087707, 0.9026336410084178, -0.267337618883517]
    tutorial.go_to_w0_detach()
    # tutorial.go_to_joint_state(detach_wheel0)

    print "============ Detaching object ..."
    tutorial.detach_object('wheel0', 4)


    print "============ Press `Enter` to go to wheel1 object"
    raw_input()
    wheel1_pos = [2.777018462599235, -1.9403007681278754, 2.057705339335132, 1.351776997117338, 1.0980508289490314, 1.9419208629705356, 0.8961924514111098]
    tutorial.go_to_w1()
    # tutorial.go_to_joint_state(wheel1_pos)

    print "============ Attaching object ..."
    tutorial.attach_object('wheel1', 4)
    print "Object attached"

    print "============ Press `Enter` to go to placement position"
    raw_input()
    detach_wheel1 = [-2.8412117332213644, -0.8680279973914196, 2.766639956107351, -1.4361070139357122, 0.3633691742618234, 0.9038608222108321, -0.17821458778484808]
    tutorial.go_to_w1_detach()
    # tutorial.go_to_joint_state(detach_wheel1)

    print "============ Detaching object ..."
    tutorial.detach_object('wheel1', 4)


    print "============ Press `Enter` to go to top object"
    raw_input()
    top_pos = [2.838910966607277, -1.875108243516905, -0.7736556348885344, -0.8728112300764034, 0.784507729561571, -1.9057429119325142, -2.840182772195697]
    # tutorial.go_to_joint_state(top_pos)
    tutorial.go_to_top()
    raw_input()
    print "============ Attaching object ..."
    tutorial.attach_object('top', 4)
    print "Object attached"

    print "============  Press `Enter` to go to placement position"
    raw_input()
    detach_top = [2.414945732149793, -1.4984348557319669, 1.6105012683134586, 1.4422988405977757, -1.5044930583570189, -1.6187787856485503, -2.4218961878085894]
    # tutorial.go_to_joint_state(detach_top)
    tutorial.go_to_top_detach()

    print "============ Detaching object ..."
    tutorial.detach_object('top', 4)

    print "============ Press `Enter` to go to screw"
    raw_input()
    tutorial.go_to_screw()
    raw_input()
    print "============ Attaching object ..."
    tutorial.attach_object('screw', 4)
    print "Object attached"

    print "============  Press `Enter` to go to placement position"
    raw_input()
    detach_top = [2.414945732149793, -1.4984348557319669, 1.6105012683134586, 1.4422988405977757, -1.5044930583570189, -1.6187787856485503, -2.4218961878085894]
    # tutorial.go_to_joint_state(detach_top)
    tutorial.go_to_screw_detach()

    print "============ Detaching object ..."
    tutorial.detach_object('screw', 4)

    print "============ The end! Press `Enter` to go to start position"
    raw_input()
    tutorial.go_to_joint_state(start)

    print "============ Press `Enter` to all remove objects from the planning scene"
    raw_input()
    tutorial.remove_objects()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
