#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

import os
import sys

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import all the necessary ROS message types:
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
import math
import numpy as np
import time

class SearchActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.ctrl_c = False
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()
        
        self.robot_odom = TB3Odometry()

        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        self.at_beacon = False

        self.scan_subscriber = rospy.Subscriber("/scan",
            LaserScan, self.scan_callback)
        self.turn_vel_slow = 0.1
        self.captured_image = False
        self.start = False
        self.detected_beacon = False
        self.ready_for_scan = False
        self.box_color = 10
        self.cam_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
                                               Image, self.camera_callback)
        self.waiting_for_initial_image = True
        self.cvbridge_interface = CvBridge()

        self.m00 = 0
        self.m00_min = 100000
        self.odom_startup = True
        self.front_arc = 0

        self.lower = [(115, 224, 100), (0, 185, 100), (55, 150, 100), (75, 150, 100), (145, 175, 100), (30, 130, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (60, 255, 255), (100, 255, 255), (155, 255, 255), (35, 255, 255)]

        self.min_distance = 0.4
        self.centred_beacon = False

        self.robot_controller = MoveTB3()
        self.arc_angles = np.arange(-20, 21)

        self.min_distance_right = 0
        self.min_distance_left = 0
    
    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.check_front_arc = scan_data.ranges[18:23]
        self.min_distance = front_arc.min()
        self.object_angle = self.arc_angles[np.argmin(front_arc)]
        right = front_arc[26:] 
        left = front_arc[:15]
        self.min_distance_right = right.min()
        self.min_distance_left = left.min()
    
    def show_and_save_image(self, img, img_name):
        base_image_path = "/home/student/catkin_ws/src/team21/src"
        full_image_path = os.path.join(base_image_path, "{}.jpg".format(img_name))

        cv2.imwrite(full_image_path, img)

    def callback_function(self, odom_data):
        # obtain the orientation co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w
        # obtain the position co-ords:
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y
        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw
        if self.odom_startup:
            self.odom_startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def camera_callback(self, img_data):
        if self.start:
            if self.waiting_for_initial_image:
                for i in range(6):
                    try:
                        cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
                    except CvBridgeError as e:
                        print(e)
                    
                    height, width, channels = cv_img.shape
                    crop_width = width - 800
                    crop_height = 400
                    crop_x = int((width/2) - (crop_width/2))
                    crop_y = int((height/2) - (crop_height/2))

                    crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
                    hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

                    lower = self.lower[i]
                    upper = self.upper[i]
                    mask = cv2.inRange(hsv_img, lower, upper)
                    res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

                    m = cv2.moments(mask)
                    self.m00 = m['m00']
                    self.cy = m['m10'] / (m['m00'] + 1e-5)

                    if self.m00 > self.m00_min:
                        cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
                        self.box_color = i
                        self.colors = ["Blue","Red","Green","Turquoise", "Purple", "Yellow"]
                        print("SEARCH INITIATED: The target colour is {}.".format(self.colors[i]))
                        self.waiting_for_initial_image = False
                    
                    cv2.imshow('cropped image', crop_img)
                    cv2.waitKey(1)
            elif self.ready_for_scan:
                try:
                    cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
                except CvBridgeError as e:
                    print(e)
                
                height, width, channels = cv_img.shape
                crop_width = width - 800
                crop_height = 400
                crop_x = int((width/2) - (crop_width/2))
                crop_y = int((2 * (height)/3) - (( 2 * (crop_height/3))))

                crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
                hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

                lower = self.lower[self.box_color]
                upper = self.upper[self.box_color]
                mask = cv2.inRange(hsv_img, lower, upper)
                res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

                m = cv2.moments(mask)
                self.m00 = m['m00']
                self.cz = m['m01']/(m['m00']+1e-5) 
                self.cy = m['m10'] / (m['m00'] + 1e-5)

                if self.m00 > self.m00_min:
                    cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
                
                cv2.imshow('cropped image', crop_img)
                cv2.waitKey(1)

    def action_server_launcher(self, goal):
        while self.ctrl_c == False:
            if self.centred_beacon == False:
                if self.ready_for_scan == False:
                    r = rospy.Rate(10)

                    success = True
                    if goal.fwd_velocity <= 0 or goal.fwd_velocity > 0.45:
                        #print("Invalid velocity.  Select a value between 0 and 0.45 m/s.")
                        success = False
                    if goal.approach_distance <= 0.2:
                        #print("Invalid approach distance: I'll crash!")
                        success = False
                    elif goal.approach_distance > 3.5:
                        #print("Invalid approach distance: I can't measure that far.")
                        success = False

                    if not success:
                        self.actionserver.set_aborted()
                        return

                    #print("Request to move at {:.3f}m/s and stop {:.2f}m infront of any obstacles".format(goal.fwd_velocity, goal.approach_distance))

                    # Get the current robot odometry:
                    self.posx0 = self.robot_odom.posx
                    self.posy0 = self.robot_odom.posy

                    #print("The robot will start to move now...")
                    # set the robot velocity:
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    #print("Stopping for 5 secs")
                    time.sleep(5)

                    # Turn 180 degrees 
                    timeout = time.time() + 7
                    self.robot_controller.set_move_cmd(0.0, 1)
                    self.robot_controller.publish()
                    #print("First turn")
                    time.sleep(3.5)

                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    self.robot_controller.publish()
                    #print("Stopping")
                    time.sleep(2)
                    # Capture Image
                    self.start = True
                    self.ready_for_scan = True
                    self.robot_controller.set_move_cmd(0.0, -1)
                    self.robot_controller.publish()
                    #print("First turn")
                    time.sleep(3.5)
                    self.robot_controller.set_move_cmd(0.1, 0.0)
                    self.robot_controller.publish()
                    time.sleep(1)
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    print(self.x0, self.y0)
                while self.detected_beacon == False:
                    x = 0
                    y = 0
                    direction = -1
                    while x == 0:
                        if y % 9 == 0:
                            direction = direction*(-1)
                        self.robot_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                        while self.min_distance > goal.approach_distance:
                            p1 = [self.x0, self.y0]
                            p2 = [self.x, self.y]
                            distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
                            if distance < 2:
                                a = 1
                            elif self.m00 > self.m00_min:
                                # if self.x - 0.2 <= self.x0 <= self.x + 0.2:
                                #     print("It's the box!")
                                #     a = 1
                                #     self.robot_controller.set_move_cmd(0.0, 0.5)
                                #     self.robot_controller.publish()
                                #     time.sleep(0.1)
                                # else:
                                # blob detected
                                self.robot_controller.set_move_cmd(0.0, 0.0)
                                self.robot_controller.publish()
                                self.detected_beacon = True
                                x = 1
                                break
                            #print("min distance: {:.1f}", self.min_distance)
                            self.robot_controller.publish()
                            # check if there has been a request to cancel the action mid-way through:
                            if self.actionserver.is_preempt_requested():
                                #rospy.loginfo("Cancelling the camera sweep.")
                                self.actionserver.set_preempted()
                                # stop the robot:
                                self.robot_controller.stop()
                                success = False
                                # exit the loop:
                                break

                        timeout = time.time() + 0.25   # 0.25 seconds from now
                    
                        if self.min_distance_right < goal.approach_distance:
                            while self.min_distance_right < goal.approach_distance:
                                self.robot_controller.set_move_cmd(0, 2)
                                self.robot_controller.publish()
                        elif self.min_distance_left < goal.approach_distance:
                            while self.min_distance_left < goal.approach_distance:
                                self.robot_controller.set_move_cmd(0, -2)
                                self.robot_controller.publish()
                        while self.min_distance < goal.approach_distance:
                                self.robot_controller.set_move_cmd(0, 2*direction)
                                self.robot_controller.publish()
                        self.robot_controller.set_move_cmd(0, 0)
                        self.robot_controller.publish()

                        y+=1
                        
                        self.distance = sqrt(pow(self.posx0 - self.robot_odom.posx, 2) + pow(self.posy0 - self.robot_odom.posy, 2))
                        self.feedback.current_distance_travelled = self.distance
                        self.actionserver.publish_feedback(self.feedback)
                while self.centred_beacon == False and self.detected_beacon:
                    while not(self.cy >= 560-35 and self.cy <= 560+35):
                        if self.cy <= 560+60 and self.cy <= 560+11:
                            self.robot_controller.set_move_cmd(0.0, 0.1)
                            self.robot_controller.publish()
                        elif self.cy >= 560-60 and self.cy >= 560-11:
                            self.robot_controller.set_move_cmd(0.0, -0.1)
                            self.robot_controller.publish()
                    if self.cy >= 560-20 and self.cy <= 560+20:
                        print("BEACON DETECTED: Beaconing initiated.")
                        self.robot_controller.set_move_cmd(0, 0)
                        self.robot_controller.publish()
                        self.centred_beacon = True
                        break

            if self.centred_beacon:
                self.robot_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                self.robot_controller.publish()
                while self.min_distance > goal.approach_distance:
                    p1 = [self.x0, self.y0]
                    p2 = [self.x, self.y]
                    distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
                    if distance < 2:
                        self.robot_controller.set_move_cmd(0.0, 0.0)
                        self.robot_controller.publish()
                        print("Whoops, that might be the box")
                        self.robot_controller.set_move_cmd(0.0, 0.8)
                        self.robot_controller.publish()
                        time.sleep(1)
                        self.centred_beacon = False
                        self.detected_beacon = False
                        self.robot_controller.set_move_cmd(0.0, 0.0)
                        self.robot_controller.publish()
                        time.sleep(0.2)
                        break
                    elif not(self.cy >= 560-15 and self.cy <= 560+15):
                        if distance < 2:
                            self.robot_controller.set_move_cmd(0.0, 0.0)
                            self.robot_controller.publish()
                            print("Whoops, that might be the box")
                            self.robot_controller.set_move_cmd(0.0, 0.8)
                            self.robot_controller.publish()
                            time.sleep(1)
                            self.centred_beacon = False
                            self.detected_beacon = False
                            self.robot_controller.set_move_cmd(0.0, 0.0)
                            self.robot_controller.publish()
                            time.sleep(0.2)
                            break
                        elif self.m00 > self.m00_min:
                            self.centred_beacon = False
                            self.detected_beacon = True
                            self.robot_controller.set_move_cmd(0.0, 0.0)
                            time.sleep(1)
                            break
                        else:
                            print("Lost the beacon")
                            self.centred_beacon = False
                            self.detected_beacon = False
                            self.robot_controller.set_move_cmd(0.0, 0.0)
                            time.sleep(2)
                            break
                if self.min_distance <= 0.4 and distance > 2:
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    self.robot_controller.publish()
                    print("BEACONING COMPLETE: The robot has now stopped.")
                    self.ctrl_c = True
                    self.centred_beacon = False
                    break

if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()

        
        