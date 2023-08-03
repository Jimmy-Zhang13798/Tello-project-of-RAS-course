#! /usr/bin/python
import base64
from re import A
from typing import Counter, Hashable
import numpy as np
import imutils
import cv2
import sys
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
import time
from djitellopy import Tello
import math
from sensor_msgs.msg import Image
from std_msgs.msg import String,UInt32
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


class CamSubscriber(Node):
    def __init__(self):
        super().__init__('image_listener')


        self.cnt = 0
        self.a = 0
        self.b = 0
        self.c = 0
        self.high = 0
        self.distance_count=0
        self.about_200cm = 0
        self.marker_count=0
        self.moveforward = 0
        self.center_calibration1 = 0
        self.center_calibration2 = 0
        self.land = 0 
        
        self.image_sub = self.create_subscription(Image, '/camera', self.image_sub_callback, 10)
        self.publisher_control = self.create_publisher(Twist, '/control', 10)
        self.publisher_land = self.create_publisher(Empty, '/land', 10)
        print("Incomplete RECTANGLE-GATE Fucker is working")

    def image_sub_callback(self, msg):


        # Convert ROS Image message to OpenCV2
        cv2_img = self.imgmsg_to_cv2(msg)
        self.cnt += 1
        cnts,hole_centre=self.rectangle_det(cv2_img)

    def rectangle_det(self,img):

        max_y_2_index=-1
        max_y_1_index=-1

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #green
        lower_boundary1 = [40, 80, 60]
        upper_boundary1 = [60, 200, 255]
        lower1 = np.array(lower_boundary1, dtype="uint8")
        upper1 = np.array(upper_boundary1, dtype="uint8")
        mask = cv2.inRange(hsv, lower1, upper1)

        ## use a mask to filter , to get green color
        color_detected = cv2.bitwise_and(img, img, mask=mask)

        
        gray = cv2.cvtColor(color_detected, cv2.COLOR_BGR2GRAY)
        ## detected contour
        cnts, hierarchy= cv2.findContours(gray.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # this cnts is tuple，each element is a n*1*2 numpy, and we think every element is a contour
        ref_distance=182.5
        ref_length=392.0739

        index = -1
        max = 0
        # to find the biggest contour
        for c in range(len(cnts)):
            if cnts[c].shape[0] <= 2:#if there are less 2 points in this contours, then can't calculate area
                continue
            area = cv2.contourArea(cnts[c])

            if area >= max:
                max = area
                index = c



        if index >= 0:
            cnts=cnts[index]
            cnt_len = cv2.arcLength(cnts,True)# using perimeter as the parameters for approxPolyDP
            cnts=cv2.approxPolyDP(cnts,0.1*cnt_len,True)# this cnts are vertex of polygons, is a n*1*2 numpy

            cnts=cnts.reshape(cnts.shape[0], cnts.shape[2])# change n*1*2 numpy into n*2 numpy 

            cv2.drawContours(img, [cnts], -1, (0,255,0), 3)

            hole_centre = np.int0(np.mean(cnts, 0)) # hole_centre is a one dimension numpy，2 items
            marker_center_X= hole_centre[0]
            marker_center_Y= hole_centre[1]

            # draw the center of all ArUco marker on the image
            cv2.circle(img, (marker_center_X, marker_center_Y), 7, (0, 255, 0), -1)
            cv2.arrowedLine(img,(marker_center_X, marker_center_Y),(480, 240), (0, 0, 0), 7,0,0,0.3)




            if cnts.shape[0] >= 2:

                max_y_1=0
                max_y_2=0
                
                for i in range(cnts.shape[0]):#
                    cv2.drawMarker(img,(cnts[i][0], cnts[i][1]),(0, 0, 255),0,20,3)#
                    if cnts[i][1] >= max_y_1:
                        max_y_1 = cnts[i][1] 
                        max_y_1_index = i

                for j in range(cnts.shape[0]):#
                    if j == max_y_1_index :
                        continue
                    if cnts[j][1] >= max_y_2:
                        max_y_2 = cnts[j][1] 
                        max_y_2_index = j

                if max_y_1_index != -1 and max_y_2_index != -1 :

                    self.marker_count=self.marker_count+1

                    mean_distance=(ref_distance*ref_length)\
                    /math.sqrt((cnts[max_y_1_index][0]-cnts[max_y_2_index][0])**2+(cnts[max_y_1_index][1]-cnts[max_y_2_index][1])**2)
                    cv2.putText(img, "gate_mean_distance:"+str(int(mean_distance))+"cm", (20, 64), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2, cv2.LINE_AA)

                    if mean_distance < 270:
                        self.distance_count=self.distance_count+1

    
        else:
            # " No rectangle_detected "
            cv2.putText(img, " No rectangle_detected ", (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA) 


#key information from above code :
#1.marker_center_X
#2.marker_center_Y
#3.mean_distance (self.distance_count)




#################################################controlling part######################################################################


        # Because the drone flies relatively low after taking off. 
        # So in any case, fly the plane a little higher to get a better view
        if self.high == 0:
            print("let the drone move higher to get a better view, first")
            move_cmd = Twist()
            move_cmd.angular.z = 0.0
            move_cmd.linear.z = 30.0
            move_cmd.linear.x = 0.0
            move_cmd.linear.y = 0.0
            self.publisher_control.publish(move_cmd)
            time.sleep(5)
            move_cmd1 = Twist()
            move_cmd1.angular.z = 0.0
            move_cmd1.linear.z = 0.0
            move_cmd1.linear.x = 0.0
            move_cmd1.linear.y = 0.0
            self.publisher_control.publish(move_cmd1)
            self.high = 1 

            time.sleep(1)
            print("Step1: Adjust the drone horizontally, vertically!")




###################################################################################################################################
# Step1: Adjust the drone horizontally, vertically!
# Step2: Detect if the drone is close enough, 200cm is the threshold value
#        If it's not close enough, then fly to get close
#        If it's close enough, start next step
# Step3: Adjust the drone horizontally, vertically, again in this short distance
# Step4: Pass through the gate

###################################################################################################################################
        if max_y_1_index != -1 and max_y_2_index != -1:


            # Step1: Adjust the drone horizontally, vertically!
            #(480,260)
            
            # Step1: Adjust the drone horizontally, vertically!
            if self.marker_count % 6 == 0 and self.center_calibration1 == 0:
                print("----------------------------------------")

                move_cmd2 = Twist()
                move_cmd2.linear.x = 0.0
                move_cmd2.linear.y = 0.0

                # we have to know the view of drone's camera is a little bit downward 
                # so when the actual drone is towards to gate's center, 
                # the gate's center in camera's picture should be a little higher than the picture's center(360)
                # our goal is 260， and we set up this range :250-270
                # 170 250-|260|-270 320
                #########################
                if  320 <= marker_center_Y : 
                    
                    print("----move down fast")
                    move_cmd2.linear.z = -12.0

                elif  270< marker_center_Y < 320 : 
                    
                    print("----move down slowly")
                    move_cmd2.linear.z = -7.0 


                elif  170< marker_center_Y < 250 : 
                        
                    print("----move up slowly")
                    move_cmd2.linear.z = 7.0 

                elif  marker_center_Y <= 170 : 

                    print("----move up fast")
                    move_cmd2.linear.z = 12.0 

                else:# 250 <= marker_center_Y <= 270 

                    print("----Basically, drone is at the same height as the center.")
                    move_cmd2.linear.z = 0.0 

                # this "if" sentence is for right-left direction 
                # our goal is 480， and we set up this range :465-495
                # 360 420 465-|480|-495 540 600  
                ##########################
                if  600 < marker_center_X:
                    print("----clockwise rotate quickly")
                    move_cmd2.angular.z = 6.0
                
                elif 540 < marker_center_X <= 600:
                    print("----clockwise rotate slowly")
                    move_cmd2.angular.z = 5.0

                elif 495< marker_center_X <=540:
                    print("----clockwise rotate very very slowly!")
                    move_cmd2.angular.z = 4.0

                elif 420 <= marker_center_X <465 :
                    print("----counter_clockwise rotate very very slowly!")
                    move_cmd2.angular.z = -4.0

                elif 360 <= marker_center_X <420 :
                    print("----counter_clockwise rotate slowly!")
                    move_cmd2.angular.z = -5.0

                elif marker_center_X < 360 :
                    print("----counter_clockwise rotate quickly!")
                    move_cmd2.angular.z = -6.0
                    
                else:# when 465 <= marker_center_X <= 495    

                    print("----Basically, drone is towards the center ")
                    move_cmd2.angular.z = 0.0

                ##################################
                #if it's in the position horizontally, and vertically at the same time,
                # then Step1 finished!
                if (465 <= marker_center_X <= 495) and  250 <= marker_center_Y <= 270:
                    print("Step1 finished! Tello is towards gate horizontally, and vertically !!")
                    move_cmd2.angular.z = 0.0
                    move_cmd2.linear.z = 0.0

                    self.center_calibration1=1
                    print("Step2: Let the drone close to gate enough")

                self.publisher_control.publish(move_cmd2)

###################################################################################################################################
            # Step2: Detect if the drone is close enough, 200cm is the threshold value
            # If it's not close enough, then fly to get close
            # If it's close enough, start next step
            if self.center_calibration1 == 1:

                # If it's not close enough, then fly to get close
                if self.distance_count < 3 and self.moveforward == 0:
                    time.sleep(1)
                    print("----NOW! we let tello move forwards")
                    move_cmd3 = Twist()
                    move_cmd3.angular.z = 0.0
                    move_cmd3.linear.z = 0.0
                    move_cmd3.linear.x = 0.0
                    move_cmd3.linear.y = 15.0
                    self.publisher_control.publish(move_cmd3)

                    self.moveforward = 1

                # If it's close enough, let it stop, and start next step
                if self.distance_count >= 3 and self.about_200cm == 0: 
                    print("Step2 finished: it's close enough! Stop the drone! ") 
                    move_cmd4 = Twist()
                    move_cmd4.angular.z = 0.0
                    move_cmd4.linear.z = 0.0
                    move_cmd4.linear.x = 0.0
                    move_cmd4.linear.y = 0.0
                    self.publisher_control.publish(move_cmd4)

                    self.about_200cm = 1

                    time.sleep(1)
                    print("Step3: Adjust the drone horizontally, vertically, again in this short distance")


###################################################################################################################################
            # Step3: Adjust the drone horizontally, vertically, again in this short distance
            if self.marker_count % 6 == 0 and self.about_200cm == 1 and self.center_calibration2 == 0:

                #(480,210)

                move_cmd9 = Twist()
                move_cmd9.linear.x = 0.0
                move_cmd9.linear.y = 0.0
                #########################

                # just like the first adjustment, 202-218 is what we want
                #150 200-|210|-220 290
                print("----------------------------------------")
                if  290 <= marker_center_Y : 
                    
                    print("----move down fast")
                    move_cmd9.linear.z = -12.0

                elif  220< marker_center_Y < 290 : 
                    
                    print("----move down slowly")
                    move_cmd9.linear.z = -7.0 
  
                elif  150< marker_center_Y < 200 :
                        
                    print("----move up slowly")
                    move_cmd9.linear.z = 7.0 


                elif  marker_center_Y <= 150 : 

                    print("----move up fast")
                    move_cmd9.linear.z = 12.0 
                else:# whent's 200 <= marker_center_Y <= 220 

                    print("----Basically, drone is at the same height as the center.")
                    move_cmd9.linear.z = 0.0 

                ##########################

                #473-487 is want we want
                # 360 420 465-|480|-495 540 600 

                if  600 < marker_center_X:
                    print("----clockwise rotate quickly")
                    move_cmd9.angular.z = 6.0
                
                elif 540 < marker_center_X <= 600:
                    print("----clockwise rotate slowly")
                    move_cmd9.angular.z = 5.0

                elif 495< marker_center_X <=540:
                    print("----clockwise rotate very very slowly!")
                    move_cmd9.angular.z = 4.0


                elif 420 <= marker_center_X <465 :
                    print("----counter_clockwise rotate very very slowly!")
                    move_cmd9.angular.z = -4.0

                elif 360 <= marker_center_X <420 :
                    print("----counter_clockwise rotate slowly!")
                    move_cmd9.angular.z = -5.0

                elif marker_center_X < 360 :
                    print("----counter_clockwise rotate quickly!")
                    move_cmd9.angular.z = -6.0
                    
                else:# when 473 <= marker_center_X <= 487    

                    print("----Basically, drone is towards the center ")
                    move_cmd9.angular.z = 0.0


                ##################################
                #if it's in the position horizontally, and vertically at the same time,
                # then Step4 finished!
                if (465 <= marker_center_X <= 495) and  200 <= marker_center_Y <= 220:
                    print("Step3 finished! Tello is towards gate horizontally, and vertically  AGAIN!!")
                    move_cmd9.angular.z = 0.0
                    move_cmd9.linear.z = 0.0

                    self.center_calibration2 = 1
                self.publisher_control.publish(move_cmd9)
                
###################################################################################################################################
            #Step4: pass through 
            if self.center_calibration2 == 1 and self.land == 0:
                time.sleep(1)
                print("Step4: now let's pass through the gate")
                move_cmd7 = Twist()
                move_cmd7.angular.z = 0.0
                move_cmd7.linear.z = 0.0
                move_cmd7.linear.x = 0.0
                move_cmd7.linear.y = 45.0
                self.publisher_control.publish(move_cmd7)
                time.sleep(3)
                move_cmd8 = Twist()
                move_cmd8.angular.z = 0.0
                move_cmd8.linear.z = 0.0
                move_cmd8.linear.x = 0.0
                move_cmd8.linear.y = 0.0
                self.publisher_control.publish(move_cmd8)
                time.sleep(1)

                land1=Empty()
                self.publisher_land.publish(land1)
                self.land = 1 

                print("Already landed")

    ########################################################################################

        cv2.imshow("a",img)
        cv2.waitKey(1)

        return cnts,hole_centre

    #a convert function        
    def imgmsg_to_cv2(self, img_msg):
        n_channels = len(img_msg.data) // (img_msg.height * img_msg.width)
        dtype = np.uint8

        img_buf = np.asarray(img_msg.data, dtype=dtype) if isinstance(img_msg.data, list) else img_msg.data

        if n_channels == 1:
            cv2_img = np.ndarray(shape=(img_msg.height, img_msg.width),
                            dtype=dtype, buffer=img_buf)
        else:
            cv2_img = np.ndarray(shape=(img_msg.height, img_msg.width, n_channels),
                            dtype=dtype, buffer=img_buf)

        # If the byte order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            cv2_img = cv2_img.byteswap().newbyteorder()

        return cv2_img


def main():
    rclpy.init()
    cam_subscriber = CamSubscriber()

    # Spin until ctrl + c
    rclpy.spin(cam_subscriber)

    cam_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

