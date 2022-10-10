#!/usr/bin/python

import rospy
from tf import TransformListener, TransformBroadcaster
from nav_msgs.msg import Odometry

class RobotPosition2d():
    """
    Subscribe to robots position and publishes it in an easy to parse format.
    """
    RATE = 10 # [Hz]
    SOURCE_FRAME = "create1/base_link"
    TARGET_FRAME = "map"
    def __init__(self):
        self._tf_listener = TransformListener()
        self._rate = rospy.Rate(self.RATE)
        self._tf_broadcaster = TransformBroadcaster(queue_size=2)
        self._robot_position_pub = rospy.Subscriber("/create1/gts", Odometry, self._odom_cb)

    def _odom_cb(self
    , gts #type: Odometry
    ):
        now = rospy.Time.now()
        trans = gts.pose.pose.position
        trans =(trans.x, trans.y, trans.z)

        ori = gts.pose.pose.orientation
        ori = (ori.x, ori.y, ori.z, ori.w)
        self._tf_broadcaster.sendTransform(trans, ori, now, "create1/base_link","gts")

    def run(self):
        while not rospy.is_shutdown():
            self._rate.sleep()

    
if __name__ == '__main__':
    rospy.init_node('RobotPosition2d', anonymous=False, log_level=rospy.DEBUG)
    try:
        node = RobotPosition2d()
        node.run()
    except rospy.ROSInterruptException:
        exit(0)
