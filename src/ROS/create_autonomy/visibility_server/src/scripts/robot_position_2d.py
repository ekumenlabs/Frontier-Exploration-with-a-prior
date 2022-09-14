#!/usr/bin/python

import rospy
from tf import TransformListener
from geometry_msgs.msg import Point

class RobotPosition2d():
    """
    Subscribe to robots position and publishes it in an easy to parse format.
    """
    RATE = 10 # [Hz]
    TARGET_FRAME = "base_link"
    SOURCE_FRAME = "map"
    def __init__(self):
        self._tf_listener = TransformListener()
        self._rate = rospy.Rate(self.RATE)
        self._robot_position_pub = rospy.Publisher("robot_position_2d", Point, queue_size=10)

    def get_robot_position(self):
        can_transform = self._tf_listener.canTransform(self.TARGET_FRAME, self.SOURCE_FRAME, rospy.Time.now())
        if not can_transform:
            return None
        trans, _ = self._tf_listener.lookupTransform(self.TARGET_FRAME, self.SOURCE_FRAME, rospy.Time.now())
        ret = Point()
        ret.x = trans[0]
        ret.y = trans[1]
        return ret

    def run(self):
        while not rospy.is_shutdown():
            maybe_robot_position = self.get_robot_position()
            if maybe_robot_position is not None:
                self._robot_position_pub.publish(maybe_robot_position)
            self._rate.sleep()

    
if __name__ == '__main__':
    rospy.init_node('RobotPosition2d', anonymous=False, log_level=rospy.DEBUG)
    try:
        node = RobotPosition2d()
        node.run()
    except rospy.ROSInterruptException:
        exit(0)
