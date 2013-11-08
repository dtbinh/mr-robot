#!/usr/bin/env python
import roslib; roslib.load_manifest('kf_lab')
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import JointState
import tf
from tf.transformations import euler_from_quaternion
import numpy


class KFExample:
    def __init__(self,name):
        self.name = name
        self.position_stddev = 0.50 # [m]
        self.ref_frame = "/world"
        self.pose_frame = "/bubbleRob"
        rospy.init_node('kf_example')
        self.ref_frame = rospy.get_param("~ref_frame",self.ref_frame)
        self.pose_frame = rospy.get_param("~pose_frame",self.pose_frame)
        self.position_stddev = rospy.get_param("~position_stddev_m",self.position_stddev)
        rospy.loginfo("Starting KF Example" )
        self.listener = tf.TransformListener()
        self.array = Float64MultiArray()
        self.array.layout = MultiArrayLayout()
        self.array.layout.data_offset = 0
        self.array.layout.dim = [MultiArrayDimension('data',4,4)]
        self.array.data = [0.0] * 4

        # State is x, y, vx, vy
        self.state = numpy.vstack([0.0, 0.0, 0.0, 0.0])

    # Filter measurement Z = numpy.vstack([x,y])
    def filter(self,Z):
        # Implement kalman filter here
        # Prediction stage
        I = numpy.eye(4)
        self.state = I * self.state

        # Update stage
        F = numpy.mat([
            [1,0],
            [0,1],
            [0,0],
            [0,0]])
        self.state = F * Z
    
        # Now publish the result
        self.array.data = [self.state[i] for i in range(4)]
        self.pub.publish(self.array)

    def run(self):
        rate = rospy.Rate(5)
        # Required for TF to initialise
        rospy.sleep(1.0)
        self.pub = rospy.Publisher("~state",Float64MultiArray)
        while not rospy.is_shutdown():
            t = rospy.Time.now()
            self.listener.waitForTransform(self.pose_frame,self.ref_frame, t, rospy.Duration(1.0))
            ((x,y,z),rot) = self.listener.lookupTransform(self.pose_frame,self.ref_frame, t)
            euler = euler_from_quaternion(rot)
            Z = numpy.vstack([x,y])
            self.filter(Z)
            rate.sleep()

if __name__ == '__main__':
    try:
        rd = KFExample("kf_example") 
        rd.run()
    except rospy.ROSInterruptException:
        pass

