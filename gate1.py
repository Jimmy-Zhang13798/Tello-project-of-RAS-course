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
        print("Aruco Fucker is working")

        self.cnt = 0
        self.a = 0
        self.b = 0
        self.c = 0
        self.n = 0
        self.far_count=0
        self.about_200cm = 0
        self.marker_count=0
        self.moveforward = 0
        self.center_calibration1 = 0
        self.center_calibration2 = 0
        self.land = 0 
        self.high = 0

        self.image_sub = self.create_subscription(Image, '/camera', self.image_sub_callback, 10)
        self.publisher_control = self.create_publisher(Twist, '/control', 10)
        self.publisher_land = self.create_publisher(Empty, '/land', 10)

    def image_sub_callback(self, msg):


        # Convert ROS Image message to OpenCV2
        cv2_img = self.imgmsg_to_cv2(msg)

        self.cnt += 1
        self.aruco_det1(cv2_img)


    def aruco_det1(self,img):

        dist=np.array(([[-0.041948, 0.048619, -0.022789, -0.004038, 0.000000]]))
        cameramatrix=np.array([[919.424717, 0.000000, 459.655779],
        [0.000000, 911.926190, 323.551997],
        [  0.,           0.,           1.        ]])
        markerlength= 0.0755
        # define names of each possible ArUco tag OpenCV supports
        ARUCO_DICT = { "DICT_4X4_50": cv2.aruco.DICT_4X4_50, "DICT_4X4_100": cv2.aruco.DICT_4X4_100 }
        aruco_dict=cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_4X4_50"])
        parameters = cv2.aruco.DetectorParameters_create()
        corners,ids,rejected=cv2.aruco.detectMarkers(img,aruco_dict,parameters=parameters)
        # this corner is a tuple.Its length is the number of marker. And each element in this tuple is a 1*4*2 numpy
        # ids is a n*1 numpy，n=len(corner)

        if len(corners) > 0:
            self.marker_count=self.marker_count+1


#####################################################################################################################
# Mainly, we have 2 ways to get key information, for controling the drone
# 1.estimatePoseSingleMarkers: it can detect 3 axes and distance. 
#   so we can know the distance to gate, and know if the marker is on right or left
# 2.calculate the center of the gate
#####################################################################################################################



#####################################################################################################################
# 1.estimatePoseSingleMarkers

            rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, markerlength, cameramatrix, dist)
            # tvec is a n*1*3 numpy, and rvec is a n*1*3 numpy, n=number of markers
            # tvec: the markers' positions relative to your camera
            # tvec: +Z is in front of the camera, +X is to the right, and +Y is down.
            # rvec is : rotation vecter


            (rvec - tvec).any()  # get rid of that nasty numpy value array error
            
            right_count=0
            left_count=0
            mean_distance=0

            # draw axes, X：red Y:green Z:blue  
            for j in range(rvec.shape[0]):
                cv2.aruco.drawAxis(img, cameramatrix, dist, rvec[j, :, :], tvec[j, :, :], 0.05) 

                temp_Z=int(((tvec[j][0][2])) * 100 )
                mean_distance=mean_distance+temp_Z
                #temp_Z is the distance to the marker 

                if temp_Z < 220:
                    self.far_count=self.far_count+1


                # use Rodrigues to tranfer: rotation vecter to rotation matrix
                R = np.zeros((3, 3), dtype=np.float64)
                cv2.Rodrigues(rvec[j], R)
                a=np.array([[0],[0],[1]])
                m=R @ a   # m is Z axsis of marker

                if m[0, 0] < 0:
                    left_count=left_count+1
                else:
                    right_count=right_count+1


            # we take that: mean distance to several markers is, distance to gate 
            mean_distance=int(mean_distance/rvec.shape[0])
            cv2.putText(img, "gate_mean_distance:"+str(mean_distance)+"cm", (20, 64), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2, cv2.LINE_AA)
################################################################################################################################
# 2.calculate the center of the gate
            corners1 = np.concatenate(corners, axis=1)
            corners1 = corners1[0,:,:]#change it to 2 dimensions
            hole_centre1 = np.int0(np.mean(corners1, 0)) # 中心


            marker_center_X = hole_centre1[0]
            marker_center_Y = hole_centre1[1]


            cv2.circle(img, (marker_center_X, marker_center_Y), 7, (0, 255, 0), -1)
            cv2.arrowedLine(img,(marker_center_X, marker_center_Y),(480, 240), (0, 0, 0), 7,0,0,0.3)
            cv2.imshow("a",img)
            cv2.waitKey(1)
        

#key information from above code :
#1.marker_center_X
#2.marker_center_Y
#3.mean_distance (self.distance_count)
#4.left_count & right_count





##############################controlling part######################################################################

            # Because the drone flies relatively low after taking off. 
            # So in any case, fly the plane a little higher to get a better view
            if self.high == 0 :
                print("Let the drone move higher to get a better view, first")
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

####################################################################################################################
# Step1: Adjust the drone horizontally, vertically!
# Step2: Detect if the drone is close enough, 200cm is the threshold value
#        If it's not close enough, then fly to get close
#        If it's close enough, start next step
# Step3: Check and move if it's too left and right
# Step4: Adjust the drone horizontally, vertically, again in this short distance
# Step5: Pass through the gate

#####################################################################################################################

            #Step1: Adjust the drone horizontally, vertically!
            if self.marker_count % 10 == 0 and self.center_calibration1 == 0:
            #(480,260)
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
                if self.far_count < 16 and self.moveforward == 0:
                    time.sleep(1)
                    print("----NOW! we let tello move forwards")
                    move_cmd3 = Twist()
                    move_cmd3.angular.z = 0.0
                    move_cmd3.linear.z = 0.0
                    move_cmd3.linear.x = 0.0
                    move_cmd3.linear.y = 25.0
                    self.publisher_control.publish(move_cmd3)

                    self.moveforward = 1

                # If it's close enough, let it stop, and start next step
                if self.far_count >= 16 and self.about_200cm == 0:
                    print("Step2 finished: it's close enough! Stop the drone! ") 
                    move_cmd4 = Twist()
                    move_cmd4.angular.z = 0.0
                    move_cmd4.linear.z = 0.0
                    move_cmd4.linear.x = 0.0
                    move_cmd4.linear.y = 0.0
                    self.publisher_control.publish(move_cmd4)

                    self.about_200cm = 1

                    time.sleep(1)
                    print("Step3: Check and move if it's too left and right")

###################################################################################################################################
            # Step3: Check and move if it's too left and right
            if self.about_200cm == 1 and self.n == 0: 

                # because we have several markers at one picture, some maybe on left
                # some maybe on right, we vote for the final decision  

                print(left_count)
                print(right_count)
                if left_count > right_count : 
                    self.a=self.a+1
                    print("----Maybe, the Aruco is on left of the drone")

                elif right_count > left_count :
                    self.b=self.b+1
                    print("----Maybe, the Aruco is on right of the drone")
                else:
                    print("----Sorry, from this callback I can't tell: right or left")
                    self.c=self.c+1

                # if we have 5 pictures told us it's on left, then we're confident it's on left
                if self.a == 3 :
                    print("----Surely,the Aruco is on left of the drone, move drone left")
                    move_cmd5 = Twist()
                    move_cmd5.angular.z = 0.0
                    move_cmd5.linear.z = 0.0
                    move_cmd5.linear.x = -8.0
                    move_cmd5.linear.y = 0.0
                    self.publisher_control.publish(move_cmd5)
                    time.sleep(3)
                    move_cmd6 = Twist()
                    move_cmd6.angular.z = 0.0
                    move_cmd6.linear.z = 0.0
                    move_cmd6.linear.x = 0.0
                    move_cmd6.linear.y = 0.0
                    self.publisher_control.publish(move_cmd6)
                    self.n=1

                    time.sleep(1)
                    print("Step4: Adjust the drone horizontally, vertically, again")

                #if we have 5 pictures told us it's on right, then we're confident it's on right
                if self.b == 3 :
                    print("----Surely,the Aruco is on right of the drone, move drone right")
                    move_cmd5 = Twist()
                    move_cmd5.angular.z = 0.0
                    move_cmd5.linear.z = 0.0
                    move_cmd5.linear.x = 8.0
                    move_cmd5.linear.y = 0.0
                    self.publisher_control.publish(move_cmd5)
                    time.sleep(3)
                    move_cmd6 = Twist()
                    move_cmd6.angular.z = 0.0
                    move_cmd6.linear.z = 0.0
                    move_cmd6.linear.x = 0.0
                    move_cmd6.linear.y = 0.0
                    self.publisher_control.publish(move_cmd6)
                    self.n=1

                    time.sleep(1)
                    print("Step4: Adjust the drone horizontally, vertically, again")

                #if we have 3 pictures cannot tell us it's on right or left, then we think it's good now 
                if self.c == 3 :
                    
                    print("----It's good now, not too right or left!")
                    move_cmd5 = Twist()
                    move_cmd5.angular.z = 0.0
                    move_cmd5.linear.z = 0.0
                    move_cmd5.linear.x = 0.0
                    move_cmd5.linear.y = 0.0
                    self.publisher_control.publish(move_cmd5)
                    self.n=1

                    time.sleep(1)
                    print("Step4: Adjust the drone horizontally, vertically, Again!")
###################################################################################################################################
            # Step4: Adjust the drone horizontally, vertically, again in this short distance
            if self.marker_count % 10 == 0 and self.n == 1 and self.center_calibration2 == 0:
                print("----------------------------------------")
                #(480,210)

                move_cmd9 = Twist()
                move_cmd9.linear.x = 0.0
                move_cmd9.linear.y = 0.0

                #########################

                # just like the first adjustment, 202-218 is what we want
                #170 202-|210|-218 270
                
                if  270 <= marker_center_Y : 
                    
                    print("----move down fast")
                    move_cmd9.linear.z = -12.0

                elif  218< marker_center_Y < 270 : 
                    
                    print("----move down slowly")
                    move_cmd9.linear.z = -7.0 
  
                elif  170< marker_center_Y < 202 :
                        
                    print("----move up slowly")
                    move_cmd9.linear.z = 7.0 


                elif  marker_center_Y <= 170 : 

                    print("----move up fast")
                    move_cmd9.linear.z = 12.0 
                else:# whent's 202 <= marker_center_Y <= 218 

                    print("----Basically, drone is at the same height as the center.")
                    move_cmd9.linear.z = 0.0 

                ##########################

                #473-487 is want we want
                # 360 420 473-|480|-487 540 600 

                if  600 < marker_center_X:
                    print("----clockwise rotate quickly")
                    move_cmd9.angular.z = 6.0
                
                elif 540 < marker_center_X <= 600:
                    print("----clockwise rotate slowly")
                    move_cmd9.angular.z = 5.0

                elif 487< marker_center_X <=540:
                    print("----clockwise rotate very very slowly!")
                    move_cmd9.angular.z = 4.0


                elif 420 <= marker_center_X <473 :
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
                if (473 <= marker_center_X <= 487) and  202 <= marker_center_Y <= 218:
                    print("Step4 finished! Tello is towards gate horizontally, and vertically  AGAIN!!")
                    move_cmd9.angular.z = 0.0
                    move_cmd9.linear.z = 0.0

                    self.center_calibration2 = 1
                self.publisher_control.publish(move_cmd9)
                
                
###################################################################################################################################
            #Step5: pass through 
            if self.center_calibration2 == 1 and self.land == 0:
                time.sleep(1)
                print("Step5: now let's pass through the gate")
                move_cmd7 = Twist()
                move_cmd7.angular.z = 0.0
                move_cmd7.linear.z = 0.0
                move_cmd7.linear.x = 0.0
                move_cmd7.linear.y = 45.0
                self.publisher_control.publish(move_cmd7)
                time.sleep(3.1)
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

        else:
            # DRAW "NO IDS" 
            cv2.putText(img, "No Ids", (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow("a",img)
            cv2.waitKey(1)
        
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


