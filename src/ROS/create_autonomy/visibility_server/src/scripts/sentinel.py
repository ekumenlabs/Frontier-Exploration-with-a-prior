#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
from subprocess import check_call

class Sentinel():
    """
    Subscribe to robots position and publishes it in an easy to parse format.
    """
    RATE = 10 # [Hz]
    FINISHED_TOPIC = "/create1/explore/exploration_finished"
    def __init__(self):
        self._rate = rospy.Rate(self.RATE)
        self._robot_position_pub = rospy.Subscriber(self.FINISHED_TOPIC, Bool, self._finished_cb)

    def _finished_cb(self
    , _ #type: Bool
    ):
        check_call(["killall" ,"rosmaster"])

    def run(self):
        while not rospy.is_shutdown():
            self._rate.sleep()

    
if __name__ == '__main__':
    rospy.init_node('sentinel', anonymous=False, log_level=rospy.DEBUG)
    try:
        node = Sentinel()
        node.run()
    except rospy.ROSInterruptException:
        exit(0)
