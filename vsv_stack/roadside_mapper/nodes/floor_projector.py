#!/usr/bin/env python
import roslib
roslib.load_manifest('roadside_mapper')

import rospy
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2.cv as cv
from cv_bridge import CvBridge
import math
import tf
import numpy
import cv2

class FloorMapper:
    def __init__(self):
        self.proba = None
        self.info = None
        self.br = CvBridge()
        rospy.init_node('floor_projector')

        image_size = rospy.get_param("~floor_size_pix",1000)
        image_extent = rospy.get_param("~floor_size_meter",5.0)
        self.target_frame = rospy.get_param("~target_frame","/body")
        self.horizon_offset = rospy.get_param("~horizon_offset_pix",20)
        self.vertical_offset = rospy.get_param("~vertical_offset_pix",0)

        self.floor_map = cv.CreateImage( (image_size,image_size), 8, 3)
        self.x_floor = 0.0
        self.y_floor = self.floor_map.height / 2.0 + self.vertical_offset
        self.floor_scale = self.floor_map.width / image_extent
        
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher("~floor",Image)
        rospy.Subscriber("~probabilities",Image,self.store_proba)
        rospy.Subscriber("~info",CameraInfo,self.store_info)
        rospy.loginfo("Waiting for first proba and camera info")
        
        # edge detection
        self.floor_gray = cv.CreateImage((image_size,image_size), 8, 1)
        self.floor_seg = cv.CreateImage((image_size,image_size), 8, 1)
        self.floor_edge = cv.CreateImage((image_size,image_size), 8, 1)
        self.pub_edge = rospy.Publisher("~edge", Image)
        while (not rospy.is_shutdown()) and ((not self.info) or (not self.proba)):
            rospy.sleep(0.1)

    def store_proba(self,proba):
        # print "Got Image"
        if not self.info:
            return
        # print "Processing"
        self.timestamp = proba.header.stamp
        I = self.br.imgmsg_to_cv(proba,"rgb8")
        self.proba = cv.CloneMat(I)
        cv.Threshold(I,self.proba,0xFE,0xFE,cv.CV_THRESH_TRUNC)
        try:
            # (trans,rot) = self.listener.lookupTransform(proba.header.frame_id, '/world', proba.header.stamp)
            self.listener.waitForTransform(proba.header.frame_id,self.target_frame,proba.header.stamp,rospy.Duration(1.0))

            cv.Smooth(self.proba,self.proba,cv.CV_GAUSSIAN,21,21,200) # smooth the image by a Gaussian filter
            msg = self.br.cv_to_imgmsg(self.proba,encoding='rgb8')
            msg.header.stamp = proba.header.stamp
            msg.header.frame_id = self.target_frame
            
            
            self.pub.publish(msg)
            # print "Publishing image"  
            
            # Publish the edge image
            cv.CvtColor(self.proba, self.floor_gray, cv.CV_BGR2GRAY)
            cv.Threshold(self.floor_gray,self.floor_seg, 150, 255, cv2.THRESH_BINARY)
            cv.Canny(self.floor_seg,self.floor_edge,100,200,3)
            
            msg_e = self.br.cv_to_imgmsg(self.floor_edge,encoding='mono8')
            msg_e.header.stamp = proba.header.stamp
            msg_e.header.frame_id = self.target_frame
            self.pub_edge.publish(msg_e)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Exception while looking for transform"
            return
        
        
            

    def store_info(self,info):
        if not self.info:
            # assuming no distortion
            self.f = info.K[0]
            self.xc = info.K[2]
            self.yc = info.K[5]
            print("Got camera info: f %.2f C %.2f %.2f" % (self.f,self.xc,self.yc))
            self.origin = numpy.zeros((4,1))
            self.origin[3,0] = 1.0

            self.horizon_offset = int(math.ceil(info.height/2. + self.horizon_offset))
            srcpts = [[0,info.height-1],[info.width-1,info.height-1],\
                    [0,self.horizon_offset], [info.width-1,self.horizon_offset]]
            self.srcpts2d = cv.CreateMat(4,2,cv.CV_32F)
            for i in range(4):
                self.srcpts2d[i,0] = srcpts[i][0]
                self.srcpts2d[i,1] = srcpts[i][1] - self.horizon_offset
            self.dirpts3d = []
            for i in range(4):
                v3 = numpy.matrix([-(srcpts[i][0]-self.xc) / self.f, 
                    -(srcpts[i][1]-self.yc) / self.f, 1.0, 0.0]).T
                n = math.sqrt((v3.T*v3).sum())
                self.dirpts3d.append(v3/n)
            self.info = info
        # print self.dirpts3d

    def run(self):
        rospy.loginfo("Starting floor projection")
        rospy.spin()

if __name__=="__main__":
    demo = FloorMapper()
    demo.run()
