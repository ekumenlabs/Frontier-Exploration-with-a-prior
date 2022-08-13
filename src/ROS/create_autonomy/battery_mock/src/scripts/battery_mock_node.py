#!/usr/bin/env python
# license removed for brevity
from math import hypot
import rospy
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from threading import Lock
from tf import TransformListener

DEBUG = True

def distance_between_odom_msgs(
    odom1, # type: Odometry
    odom2 # type: Odometry
):
    """
    Calculate the euclidean distance in meters between two Odometry messages.
    """
    return hypot(odom1.pose.pose.position.x - odom2.pose.pose.position.x,
                 odom1.pose.pose.position.y - odom2.pose.pose.position.y)

CHARGIN_SPOT_COORDINATES = (0, 2.0) # [m] X, Y
class BatteryMocker(object):
    """Consumes time elapsed and robot odometry to estimate battery consumption
    """
    IN_CHARGING_SPOT_THRESHOLD = 0.1 # [m]
    CHARGE_DELTA = 1 # [% per sec]
    TRAVELLED_DISTANCE_CONSUMPTION = 0.02 # [% per meter travelled]
    def __init__(self):
        self._pub = rospy.Publisher('battery', BatteryState, queue_size=10)
        self._rate = rospy.Rate(10) # 10hz
        self._last_received_odom = None
        self._lock = Lock() # protects and last_received_odom
        self._travelled_distance = 0 # travelled distance since last battery estimation.
        self._odom_sub = rospy.Subscriber('/create1/odom', Odometry, self._odom_cb)
        self._last_time = rospy.Time.now()
        self._battery = BatteryState()
        self._battery.percentage = 100
        self._battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        self._tf_listener = TransformListener()
    
    def _odom_cb(self, odom_msg):
        """
        Callback for the odometry topic"""
        with self._lock:
            if(self._last_received_odom is not None):
                self._compute_distance_travelled(odom_msg=odom_msg)
            self._last_received_odom = odom_msg
        
    
    def _in_charging_spot(self):
        """
        returns whether the robot is in it's starting spot or not
        """
        with self._lock:
            if self._last_received_odom is None:
                return False
            now = rospy.Time.now()
            try:
                self._tf_listener.waitForTransform("/map", "/create1/base_link", now, rospy.Duration(secs=4))
            except:
                return False
            if not self._tf_listener.canTransform("map", "/create1/base_link", now):
                return False
            trans, _ = self._tf_listener.lookupTransform("/map", "/create1/base_link",now)
            x_diff = trans[0] - CHARGIN_SPOT_COORDINATES[0]
            y_diff = trans[1] - CHARGIN_SPOT_COORDINATES[1]
            return hypot(x_diff, y_diff) < self.IN_CHARGING_SPOT_THRESHOLD

    def _compute_distance_travelled(self, 
        odom_msg # type: Odometry
    ):
        """
        Adds travelled distance since last odometry observation, reflecting that in the object state.
        """

        assert self._last_received_odom is not None
        self._travelled_distance += distance_between_odom_msgs(odom1=self._last_received_odom, odom2=odom_msg)

    def _estimate_battery_consumption(self):
        """
        Updates the battery state based on the travelled distance and the time elapsed since the last update.
        """
        now = rospy.Time.now()
        # compute elapsed time and update last time
        time_elapsed = (now - self._last_time).to_sec()
        # this makes no sense, but simulated time has a big delta on startup, we want to filter that.
        if time_elapsed > 10:
            time_elapsed = 0
        self._last_time = now
        # check if we have all the data we need 
        if self._last_received_odom is None:
            return
        # whether we are in the charging spot or not, act accordingly
        if self._in_charging_spot():
            if(DEBUG): 
                rospy.loginfo("In charging spot")
            # if we are in the charging spot, charge the battery
            self._battery.percentage = min(100, self._battery.percentage + time_elapsed * self.CHARGE_DELTA)
            # if the battery is full, set the status inside the msg to full
            if(self._battery.percentage == 100):
                self._battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
            else:
                # if the battery is not full, set the status inside the msg to charging
                self._battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            if(DEBUG): 
                rospy.loginfo("Not in charging spot")
            # if we are not in the charging spot, consume battery based on travelled distance and time elapsed
            self._battery.percentage = max(0, self._battery.percentage - time_elapsed * self.CHARGE_DELTA)
            self._battery.percentage = max(0, self._battery.percentage - self._travelled_distance * self.CHARGE_DELTA)
            self._battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        # reset travelled distance 
        with self._lock:
            self._travelled_distance = 0
        

    def run(self):
        """
        Main entrypoint for the mocker.
        """
        while not rospy.is_shutdown():
            self._estimate_battery_consumption()
            battery = self._battery
            if DEBUG:
                rospy.loginfo("Battery: {}%".format(battery.percentage))
            self._pub.publish(battery)
            self._rate.sleep()


    
if __name__ == '__main__':
    rospy.init_node('BatteryMock', anonymous=False, log_level=rospy.DEBUG)
    try:
        battery_estimator = BatteryMocker()
        battery_estimator.run()
    except rospy.ROSInterruptException:
        pass
