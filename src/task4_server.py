#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary sROS message types:
from com2009_actions.msg import SearchFeedback, SearchResult, SearchAction
from sensor_msgs.msg import LaserScan

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

# Import some other useful Python Modules
from math import sqrt, pow, pi
import numpy as np
import time

class SearchActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/task4_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.scan_subscriber = rospy.Subscriber("/scan",
            LaserScan, self.scan_callback)

        self.robot_odom = TB3Odometry()

        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)
        # allocate variables for "current" and "starting" robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.init_x =  0.0
        self.init_y = 0.0
        self.init_yaw = 0.0
        self.startup = True

        self.min_small_front_arc = 0.0
        self.min_bigger_front_arc = 0.0

        self.scan_subscriber = rospy.Subscriber("/scan",
            LaserScan, self.scan_callback)

        self.min_distance = 0.5
        self.wall_distance = 0.5

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        self.arc_angles = np.arange(-20, 21)

        self.min_distance_right = 0
        self.min_distance_left = 0
        self.left_middle = 0
        self.right_middle = 0
        self.min_distance_bigger_right = 0.0
        self.min_distance_bigger_left = 0.0
    
    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:21] #original 21
        right_arc = scan_data.ranges[-20:] #original -20
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_distance = front_arc.min()
        self.object_angle = self.arc_angles[np.argmin(front_arc)]
        small_front_arc = front_arc[6:37] 
        self.min_small_front_arc = small_front_arc.min()
        right = front_arc[26:] 
        left = front_arc[:15]

        bigger_left_arc = scan_data.ranges[0:30]
        bigger_right_arc = scan_data.ranges[-29:]
        bigger_front_arc = np.array(bigger_left_arc[::-1] + bigger_right_arc[::-1])
        self.min_bigger_front_arc = bigger_front_arc.min()
        right_bigger = bigger_front_arc[31:]
        left_bigger = bigger_front_arc[:28]
        self.min_distance_bigger_right = right_bigger.min()
        self.min_distance_bigger_left = left_bigger.min()

        middle = front_arc[17:23]
        self.left_middle = middle[0]
        self.right_middle = middle[-1]
        self.min_distance_right = right.min()
        self.min_distance_left = left.min()
    
    def callback_function(self,odom_data):
        # obtain the orientation co-ords:
        x = odom_data.pose.pose.orientation.x
        y = odom_data.pose.pose.orientation.y
        z = odom_data.pose.pose.orientation.z
        w = odom_data.pose.pose.orientation.w

        # obtain the position co-ords:
        self.x = odom_data.pose.pose.position.x
        self.y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch yaw (theta_x, theta_y, theta_z):
        (roll, pitch, self.yaw) = euler_from_quaternion([x, y, z, w],'sxyz')

        # set the initial robot pose if this node has just been launched
        if self.startup:
            # don't initialise again:
            self.startup = False

            # set the robot starting position:
            self.init_x = self.x
            self.init_y = self.y
            self.init_yaw = self.yaw
    
    def action_server_launcher(self, goal):
        r = rospy.Rate(10)

        success = True
        if goal.fwd_velocity <= 0 or goal.fwd_velocity > 0.45:
            print("Invalid velocity.  Select a value between 0 and 0.45 m/s.")
            success = False
        if goal.approach_distance <= 0.1:
            print("Invalid approach distance: I'll crash!")
            success = False
        elif goal.approach_distance > 3.5:
            print("Invalid approach distance: I can't measure that far.")
            success = False

        if not success:
            self.actionserver.set_aborted()
            return

        print("Request to move at {:.3f}m/s and stop {:.2f}m infront of any obstacles".format(goal.fwd_velocity, goal.approach_distance))

        # Get the current robot odometry:
        self.posx0 = self.robot_odom.posx
        self.posy0 = self.robot_odom.posy

        print("The robot will start to move now...")
        # set the robot velocity:

        self.robot_controller.set_move_cmd(0, 0)
        self.robot_controller.publish()
        time.sleep(3)
        print("Stopping for 3 seconds")
        self.robot_controller.set_move_cmd(goal.fwd_velocity, 0.0)
        
        z = 8
        y = 0
        while y <= 18:
            print(y)
            self.robot_controller.set_move_cmd(goal.fwd_velocity, 0.0)
            self.robot_controller.publish()
            time.sleep(2)
            while self.min_bigger_front_arc > goal.approach_distance:
                # print("min distance: {:.1f}", self.min_distance)
                self.robot_controller.publish()
                # check if there has been a request to cancel the action mid-way through:
                if self.actionserver.is_preempt_requested():
                    rospy.loginfo("Cancelling the camera sweep.")
                    self.actionserver.set_preempted()
                    # stop the robot:
                    self.robot_controller.stop()
                    self.wall_distance = self.min_distance
                    # exit the loop:
                    break

            self.robot_controller.set_move_cmd(0, 0.0)
            self.robot_controller.publish()
            self.robot_controller.publish()
            self.robot_controller.publish()
            time.sleep(1)

            # if (y == 0 or y == 4 or y == 6 or y == 7):
            #     # Turn right
            #     # self.robot_controller.set_move_cmd(0, -0.4)
            #     # self.robot_controller.publish()
            #     print("Turning right now on turn: {:.0f}".format(y))
            #     # time.sleep(3.5)
            #     self.robot_controller.set_move_cmd(0, -0.4)
            #     self.robot_controller.publish()
            #     # while abs(self.init_yaw - self.yaw) <= 1.52:
            #     #     continue
            #     while self.min_distance <= self.wall_distance:
            #         continue
            #     self.init_yaw = self.yaw
            #     self.robot_controller.set_move_cmd(0, 0)
            #     self.robot_controller.publish()
            # else:
            #     # Turn left
            #     print("Turning left now on turn: {:.0f}".format(y))
            #     self.robot_controller.set_move_cmd(0, 0.4)
            #     self.robot_controller.publish()
            #     # while abs(self.init_yaw - self.yaw) <= 1.52:
            #     #     continue
            #     while self.min_distance <= self.wall_distance:
            #         continue
            #     self.init_yaw = self.yaw
            #     self.robot_controller.set_move_cmd(0, 0)
            #     self.robot_controller.publish()

            if self.min_small_front_arc <= goal.approach_distance:
                if (y == 0 or y == 6 or y == 10 or y == 12 or y == 15 or y == 16):
                    # Turn right
                    # self.robot_controller.set_move_cmd(0, -0.4)
                    # self.robot_controller.publish()
                    print("Turning right now on turn: {:.0f}".format(y))
                    # time.sleep(3.5)
                    self.robot_controller.set_move_cmd(0, -0.3)
                    self.robot_controller.publish()
                    while abs(self.init_yaw - self.yaw) <= 1.52:
                        continue
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    self.init_yaw = self.yaw
                    # time.sleep(1)
                elif (y == 2):
                    # Turn left
                    self.robot_controller.set_move_cmd(0.0, 0.3)
                    self.robot_controller.publish()
                    print("Turning left now on turn: {:.0f}".format(y))
                    # time.sleep(3.5)
                    while abs(self.init_yaw - self.yaw) <= 1.49:
                        continue
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    self.init_yaw = self.yaw
                    # time.sleep(1)
                elif (y == 3):
                    self.robot_controller.set_move_cmd(0.0, -0.3)
                    self.robot_controller.publish()
                    print("Turning right now on turn: {:.0f}".format(y))
                    # time.sleep(3.9)
                    while abs(self.init_yaw - self.yaw) <= 1.52:
                        continue
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    self.init_yaw = self.yaw
                    # time.sleep(1)
                elif (y == 4):
                    # Turn right
                    self.robot_controller.set_move_cmd(0.0, -0.3)
                    self.robot_controller.publish()
                    print("Turning right now on turn: {:.0f}".format(y))
                    # time.sleep(3.6)
                    while abs(self.init_yaw - self.yaw) <= 1.52:
                        continue
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    self.robot_controller.publish()
                    self.init_yaw = self.yaw
                    # time.sleep(1)
                elif (y == 5):
                    print("Turning right now on turn: {:.0f}".format(y))
                    # time.sleep(3.5)
                    self.robot_controller.set_move_cmd(0, 0.3)
                    self.robot_controller.publish()
                    while abs(self.init_yaw - self.yaw) <= 1.50:
                        continue
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0, 0)
                    self.robot_controller.publish()
                    self.init_yaw = self.yaw
                    # time.sleep(1)
                elif (y == 7):
                    print("Turning right now on turn: {:.0f}".format(y))
                    # time.sleep(3.5)
                    self.robot_controller.set_move_cmd(0, -0.3)
                    self.robot_controller.publish()
                    while abs(self.init_yaw - self.yaw) <= 1.53:
                        continue
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    self.init_yaw = self.yaw
                else:
                    # Turn left
                    print("Turning left now on turn: {:.0f}".format(y))
                    self.robot_controller.set_move_cmd(0.0, 0.3)
                    self.robot_controller.publish()
                    while abs(self.init_yaw - self.yaw) <= 1.52:
                        continue
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    self.init_yaw = self.yaw
                    # time.sleep(1)


                self.robot_controller.set_move_cmd(0.0, 0.0)
                self.robot_controller.publish()
                self.robot_controller.publish()
                y+=1
            elif self.min_bigger_front_arc <= goal.approach_distance:
                if self.min_distance_bigger_right < goal.approach_distance:
                    while self.min_distance_right < goal.approach_distance:
                        self.robot_controller.set_move_cmd(0, 0.2)
                        self.robot_controller.publish()
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                elif self.min_distance_bigger_left < goal.approach_distance:
                    while self.min_distance_bigger_left < goal.approach_distance:
                        self.robot_controller.set_move_cmd(0, -0.2)
                        self.robot_controller.publish()
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
            
            self.distance = sqrt(pow(self.posx0 - self.robot_odom.posx, 2) + pow(self.posy0 - self.robot_odom.posy, 2))
            # populate the feedback message and publish it:
            # self.feedback.current_distance_travelled = self.distance
            self.actionserver.publish_feedback(self.feedback)

        # if success:
        #     rospy.loginfo("approach completed sucessfully.")
        #     self.result.total_distance_travelled = self.distance
        #     self.result.closest_object_distance = self.min_distance
        #     self.result.closest_object_angle = self.object_angle

        #     self.actionserver.set_succeeded(self.result)
        #     self.robot_controller.stop()
            
if __name__ == '__main__':
    rospy.init_node("task4_action_server")
    SearchActionServer()
    rospy.spin()
