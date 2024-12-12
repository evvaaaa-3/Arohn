#!/usr/bin/env python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import sys
import argparse
import numpy as np
import cv2
import os, glob, time

from ooooo import *
distance = 0
global chalja
chalja = 0.0
pub = rospy.Publisher("/cmd_vel", Twist,queue_size=10)

#ROS_TOPIC = "/kinect2/sd/image_ir"  #'/mrt/camera/color/image_raw'
ROS_TOPIC = "/kinect2/hd/image_color"

class ImageSubscriber:

    """Subscribes to ROS Topic and calls image_callback"""

    def __init__(self, image_topic):
        """

        :image_topic: string

        """
        rospy.init_node("image_sub", anonymous=True)
        self.br = CvBridge()
        self.sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        # Blue color in BGR
        self.color = (255, 255, 0)
        self.org = (50, 50)
        self.fontScale = 1
        self.thickness = 2
        self.vid_file = cv2.VideoWriter(
            "arrow.mp4", cv2.VideoWriter_fourcc(*"MP4V"), 10, (640, 480)
        )
        rospy.spin()
        print("all done!")
        self.vid_file.release()
        cv2.destroyAllWindows()

    def image_callback(self, data):
        """Converts ROS Image, passes to arrow_detect and displays detected

        :data: Image
        :returns: None

        """
        global chalja
        cv_img = self.br.imgmsg_to_cv2(data)
        found, theta, orient, direction, output, distance = arrow_detect(cv_img)
        print("shape: ", output.shape)

        if direction == 1:
            direction = "Right"
        elif direction is None:
            direction = "not found"
        else:
            direction = "Left"
            
        # font
        font = cv2.FONT_HERSHEY_SIMPLEX

        # org
        org = (50, 50)

        # fontScale
        fontScale = 1

        # Blue color in BGR
        color = (255, 0, 0)

        # Line thickness of 2 px
        thickness = 2
        
        final_direction = direction
            
        if abs(orient) == 90:
            
            if distance > 200:
                msg = Twist()
                msg.angular.z = -0.6
                msg.linear.x = 0.0
                pub.publish(msg)
            else:
                # msg = Twist()
                # msg.angular.z = 0.0
                # pub.publish(msg)
                if final_direction == "Left" and 60<distance<200:
                    msg = Twist()
                    i=0
                    # while(i<100000):
                    #     msg.linear.x = -0.5
                    #     pub.publish(msg)
                    #     i+=1
                    #     print(i)
                    chalja = -0.5
                    msg.linear.x = chalja
                    pub.publish(msg)
                elif final_direction == "Right" and 60<distance<200:
                    msg = Twist()
                    i=0
                    # while(i<100000):
                    #     msg.linear.x = 0.5
                    #     pub.publish(msg)
                    #     i+=1
                    #     print(i)
                    chalja = 0.5
                    msg.linear.x = chalja
                    pub.publish(msg)
                else:
                    msg = Twist()
                    msg.linear.x = 0
                    pub.publish(msg)
            


            
            output = cv2.putText(
                output,
                final_direction + " \n" + str(orient),
                org,
                font,
                fontScale,
                color,
                thickness,
                cv2.LINE_AA,
            )

            print(f"Turn {final_direction}")
            print(distance)
            output = cv2.putText(output,
                                str(round(distance, 2))+"cm",
                                (50,100),
                                font,
                                fontScale,
                                color,
                                thickness,
                                cv2.LINE_AA)
            
        
        
        else:
            output = cv2.putText(
                output,
                "Not Detected",
                org,
                font,
                fontScale,
                color,
                thickness,
                cv2.LINE_AA,
            )
            msg = Twist()
            msg.angular.z = 0
            msg.linear.x=chalja
            pub.publish(msg)
        
        self.vid_file.write(output)
        cv2.imshow("Arrow", output)
        cv2.waitKey(20)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t", "--topic", help="ROS Topic to subscribe to", default=ROS_TOPIC
    )
    args = parser.parse_args()

    subscriber = ImageSubscriber(args.topic)
    # cv2.destroyAllWindows()
    
