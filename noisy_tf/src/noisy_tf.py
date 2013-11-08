#!/usr/bin/python
import roslib; roslib.load_manifest('noisy_tf')
import rospy
import math
import numpy
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from_frame = "/world"
to_frame = "/noisy_world"
broadcaster = tf.TransformBroadcaster()

if __name__ == '__main__':
    try:
        rospy.init_node("noisy_tf")
        from_frame = rospy.get_param("~from_frame",from_frame)
        to_frame = rospy.get_param("~to_frame",to_frame)
        angular_noise = rospy.get_param("~angular_noise_deg",0.0)
        angular_noise = angular_noise * math.pi/180.
        position_noise = rospy.get_param("~position_noise_m",0.0)
        rospy.loginfo("Started Noisy TF broadcaster")
        rate = rospy.Rate(10);
        while not rospy.is_shutdown():
            t = rospy.Time.now()
            P = position_noise*numpy.random.randn(1,3)
            E = angular_noise*numpy.random.randn(1,3)
            Q = quaternion_from_euler(E[0,0],E[0,1],E[0,2])
            broadcaster.sendTransform((P[0,0],P[0,1],P[0,2]), Q,t,from_frame,to_frame)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

