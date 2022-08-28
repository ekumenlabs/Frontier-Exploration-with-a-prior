#!/usr/bin/env python
import rospy
from sensor_msgs.msg import BatteryState
from threading import Lock
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

DEBUG = True

class BatteryMonitor(object):
    """
    Listens to the battery state, and sends the robot home when the battery is below the threshold
    """
    BATTERY_LOW_THRESHOLD = 30 # [% of full battery capacity]
    def __init__(self):
        self._rate = rospy.Rate(10) # 10hz
        self._lock = Lock() # protects self._battery
        self._battery_sub = rospy.Subscriber('/battery', BatteryState, self._battery_state_cb)
        self._going_home = False # whether the robot is already going home or not
        self._battery = None # type: BatteryState
        self._move_base_client = actionlib.SimpleActionClient('/create1/move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()
    
    def _battery_state_cb(self, battery_msg):
        """
        Callback for the battery topic
        """
        with self._lock:
            self._battery = battery_msg
        
    def _maybe_send_home(self):
        """
        Sends or not the robot home, depending if the robot is already going there, or if the battery is low or not
        """
        if self._battery is None:
            return
        if self._battery.percentage <= 0:
            self._move_base_client.cancel_all_goals()
        elif not self._going_home and self._battery.percentage < self.BATTERY_LOW_THRESHOLD:
            self._send_home()


    def _send_home(self):
        """
        sends the robot home (0,0) coordinates in map frame.
        """
        rospy.loginfo("Sending home")
        assert not self._going_home
        self._move_base_client.cancel_all_goals()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 0
        goal.target_pose.pose.position.y = 2
        goal.target_pose.pose.orientation.w = 1.0
        self._going_home = True
        self._move_base_client.send_goal_and_wait(goal)
        self._going_home = False


    def run(self):
        """
        Main entrypoint for the monitor.
        """
        while not rospy.is_shutdown():
            self._maybe_send_home()
            self._rate.sleep()


    
if __name__ == '__main__':
    rospy.init_node('BatteryMonitor', anonymous=False, log_level=rospy.DEBUG)
    try:
        battery_estimator = BatteryMonitor()
        battery_estimator.run()
    except rospy.ROSInterruptException:
        pass
