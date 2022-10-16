#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from math import hypot
from std_msgs.msg import Float32

def compute_distance_between_odom_msgs(odom1,#type: Odometry
                                        odom2 #type: Odometry
                                        ):
    x1 = odom1.pose.pose.position.x
    x2 = odom2.pose.pose.position.x
    y1 = odom2.pose.pose.position.y
    y2 = odom2.pose.pose.position.y
    x_diff = x1 - x2
    y_diff = y1 - y2
    return hypot(x_diff, y_diff)

class RobotPosition2d():
    """
    Subscribe to robots position and publishes it in an easy to parse format.
    """
    RATE = 10 # [Hz]
    def __init__(self):
        self._rate = rospy.Rate(self.RATE)
        self._robot_position_pub = rospy.Subscriber("/create1/gts", Odometry, self._odom_cb)
        self._last_position = None
        self._cumulative_distance = 0
        self._travelled_distance_pub = rospy.Publisher("/create1/travelled_distance", Float32, queue_size=1)


    def _odom_cb(self
    , odom #type: Odometry
    ):
        if self._last_position is None:
            self._last_position = odom
            return
        self._cumulative_distance += compute_distance_between_odom_msgs(self._last_position, odom)
        self._last_position = odom
        msg = Float32()
        msg.data = self._cumulative_distance
        self._travelled_distance_pub.publish(msg)
    

    def run(self):
        rospy.spin()

    
if __name__ == '__main__':
    rospy.init_node('RobotPosition2d', anonymous=False, log_level=rospy.DEBUG)
    try:
        node = RobotPosition2d()
        node.run()
    except rospy.ROSInterruptException:
        exit(0)
