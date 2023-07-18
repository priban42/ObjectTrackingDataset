#!/usr/bin/env python

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import os
import pickle
import csv

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        #self.move_group.allow_replanning(True)

    def go_to_joint_state(self):

        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):

        move_group = self.move_group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)

        move_group.stop()

        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):

        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

        return plan, fraction

    def vect_to_pose(self, vect):
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position.x = vect[0]
        pose_goal.position.y = vect[1]
        pose_goal.position.z = vect[2]
        pose_goal.orientation.w = vect[3]
        pose_goal.orientation.x = vect[4]
        pose_goal.orientation.y = vect[5]
        pose_goal.orientation.z = vect[6]


        return pose_goal

    def import_blender_camera_trajectory(self, NUMBER = 0):
        PATH = "/home/bagr/PycharmProjects/ObjectTrackingDataset/dataset/trajectory_" + str(NUMBER)
        ret = []
        with open(PATH + "/blender_camera_trajectory.csv", 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                if (row[0].isnumeric()):
                    ret.append(list(map(float, row[1:])))
        return ret

    def plan_path_from_file(self, NUMBER = 0):

        move_group = self.move_group
        vects = self.import_blender_camera_trajectory(NUMBER)
        waypoints = []
        for vect in vects:
            pose = self.vect_to_pose(vect)
            waypoints.append(pose)
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 50.0)
        #(plan, fraction) = move_group.com

        return plan, fraction

    def manual_plan_cartesian_path(self, scale=1):
        move_group = self.move_group
        waypoints = []
        while input("press enter to add new waypoint or (q + enter) to finish") == '':
            wpose = move_group.get_current_pose().pose
            waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        return plan, fraction


    def display_trajectory(self, plan):

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)


    def execute_plan(self, plan):

        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def export_plan(self, plan, NUMBER = 0):
        PATH = "/home/bagr/PycharmProjects/ObjectTrackingDataset/dataset/trajectory_" + str(NUMBER)
        file_path = os.path.join(PATH, "robot_trajectory.p")
        with open(file_path, 'wb') as file_save:
            pickle.dump(plan, file_save)

    def export_reduced_plan(self, plan, NUMBER = 0):
        reduced_plan = []
        for point in plan.joint_trajectory.points:
            reduced_plan.append([point.positions, point.time_from_start.nsecs])
        PATH = "/home/bagr/PycharmProjects/ObjectTrackingDataset/dataset/trajectory_" + str(NUMBER)
        file_path = os.path.join(PATH, "reduced_robot_trajectory.p")
        with open(file_path, 'wb') as file_save:
            pickle.dump(reduced_plan, file_save)

    def import_plan(self, name = "tmp_plan.p"):
        file_path = os.path.join("/home/bagr/ws_moveit/src/my_tests/saved_paths", name)
        with open(file_path, 'rb') as file_open:
            loaded_plan = pickle.load(file_open)
            #loaded_plan = yaml.load(file_open)
        return loaded_plan


def main():
    try:

        tutorial = MoveGroupPythonInterfaceTutorial()
        #input("Press `Enter`")
        #tutorial.go_to_joint_state()

        #input("Press `Enter`")
        #tutorial.go_to_pose_goal()

        #input("press enter to start recording waypoints")
        #cartesian_plan, fraction = tutorial.plan_cartesian_path()
        #cartesian_plan, fraction = tutorial.manual_plan_cartesian_path()
        cartesian_plan, fraction = tutorial.plan_path_from_file(0)
        tutorial.export_plan(cartesian_plan, 0)
        tutorial.export_reduced_plan(cartesian_plan, 0)

        #cartesian_plan = tutorial.import_plan()
        #input("Press `Enter`")
        tutorial.display_trajectory(cartesian_plan)

        #input("press enter to execute")
        #tutorial.display_trajectory(cartesian_plan)
        #tutorial.execute_plan(cartesian_plan)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
