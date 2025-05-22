
# --- ros_bridge.py ---
import rospy
from sensor_msgs.msg import NavSatFix
from PyQt5.QtCore import QObject, pyqtSignal

class ROSBridge(QObject):
    new_fix = pyqtSignal(str, float, float)  # topic, lat, lon

    def __init__(self, topics):
        super().__init__()
        self.subscribers = []
        for topic in topics:
            sub = rospy.Subscriber(topic, NavSatFix, self.callback, callback_args=topic)
            self.subscribers.append(sub)

    def callback(self, msg, topic):
        if msg.status.status >= 0:  # only valid fixes
            self.new_fix.emit(topic, msg.latitude, msg.longitude)

