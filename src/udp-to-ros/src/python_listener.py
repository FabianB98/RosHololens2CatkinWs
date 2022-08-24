
import roslib
roslib.load_manifest('data-listener')
import rospy
import math
import tf


if __name__ == '__main__':
    rospy.init_node('python_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/ART_pen', '/markers', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        print(trans)

        rate.sleep()
