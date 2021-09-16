#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import os

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from move_tb3 import MoveTB3

import time

class colour_search(object):

    def __init__(self):
        rospy.init_node('task2')
        self.start = False
        self.captured_image = False
        self.ready_for_scan = False
        self.box_color = 10
        self.cam_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
                                               Image, self.camera_callback)
        self.waiting_for_initial_image = True
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.turn_vel_fast = 0.5
        self.turn_vel_slow = 0.1
        #self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 100000

        self.lower = [(115, 224, 100), (0, 185, 100), (55, 150, 100), (75, 150, 100), (145, 175, 100), (30, 130, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (60, 255, 255), (100, 255, 255), (155, 255, 255), (35, 255, 255)]

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def show_and_save_image(self, img, img_name):
        base_image_path = "/home/student/catkin_ws/src/team21/src"
        full_image_path = os.path.join(base_image_path, "{}.jpg".format(img_name))

        cv2.imwrite(full_image_path, img)

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
                crop_y = int((height/2) - (crop_height/2))

                crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
                hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

                lower = self.lower[self.box_color]
                upper = self.upper[self.box_color]
                mask = cv2.inRange(hsv_img, lower, upper)
                res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

                m = cv2.moments(mask)
                self.m00 = m['m00']
                self.cy = m['m10'] / (m['m00'] + 1e-5)

                if self.m00 > self.m00_min:
                    cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
                
                cv2.imshow('cropped image', crop_img)
                cv2.waitKey(1)

            

    def main(self):
        
        self.robot_controller.set_move_cmd(0.0, 0.0)
        self.robot_controller.publish()
        #print("Stopping for 5 secs")
        time.sleep(5)

        # Turn 180 degrees 
        timeout = time.time() + 7
        self.robot_controller.set_move_cmd(0.0, 0.5)
        self.robot_controller.publish()
        #print("First turn")
        time.sleep(7)

        self.robot_controller.set_move_cmd(0.0, 0.0)
        self.robot_controller.publish()
        self.robot_controller.publish()
        #print("Stopping")
        time.sleep(3)
        # Capture Image
        self.start = True

        # Turn 180 degrees in the opposite direction
        timeout = time.time() + 7.1
        self.robot_controller.set_move_cmd(0.0, -0.5)
        self.robot_controller.publish()
        self.robot_controller.publish()
        #print("Second turn")
        time.sleep(7)

        self.robot_controller.set_move_cmd(0.0, 0.0)
        self.robot_controller.publish()
        self.robot_controller.publish()
        #print("Stopping")
        time.sleep(3)

        # Move forwards to x position
        timeout = time.time() + 5
        self.robot_controller.set_move_cmd(0.2, 0.0)
        self.robot_controller.publish()
        self.robot_controller.publish()
        #print("Moving Forwards")
        time.sleep(5)

        self.robot_controller.set_move_cmd(0.0, 0.0)
        self.robot_controller.publish()
        self.robot_controller.publish()
        #print("Stopping")
        time.sleep(3)

        # turn 90 degrees right
        timeout = time.time() + 3
        self.robot_controller.set_move_cmd(0.0, -0.5)
        self.robot_controller.publish()
        self.robot_controller.publish()
        #print("Third turn")
        time.sleep(3.9)


        self.robot_controller.set_move_cmd(0.0, 0.0)
        #print("Stopping")
        self.robot_controller.publish()
        self.robot_controller.publish()
        self.ready_for_scan = True
        time.sleep(3)

        # Scan the all the pillars

        ##########################
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        while not self.ctrl_c:

            #Start moving 180 degrees left and detect the pillar with the correct colour
            if self.stop_counter > 0:
                self.stop_counter -= 1

            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                        self.stop_counter = 20
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'
                
            if self.move_rate == 'fast':
               #print("MOVING FAST: I can't see anything at the moment (blob size = {:.0f}), scanning the area...".format(self.m00))
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == 'slow':
                #print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == 'stop' and self.stop_counter > 0:
                print("SEARCH COMPLETE: The robot is now facing the target pillar.")
                self.robot_controller.set_move_cmd(0.0, 0.0)
                break
            else:
                #print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            
            self.robot_controller.publish()
            self.rate.sleep()
            
if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass