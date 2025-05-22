# navsat_map_viewer/
#   ├── main.py              # App entry point
#   ├── ros_bridge.py        # ROS subscribers and callbacks
#   ├── map_widget.py        # PyQt5 map window with Folium
#   ├── map_logic.js         # Leaflet JS for measuring
#   └── ui_controls.py       # Topic selection and GUI buttons

# --- main.py ---
from PyQt5.QtWidgets import QApplication
from map_widget import MapWindow
import sys

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MapWindow()
    window.show()
    sys.exit(app.exec_())


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


# --- map_widget.py ---
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QCheckBox, QPushButton, QHBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QTimer
from folium import Map, Marker
import os, tempfile
from ros_bridge import ROSBridge

class MapWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NavSatFix Viewer")
        self.topics = ["/gps/fix", "/other/fix"]
        self.active_topics = set(self.topics)

        self.map_center = [0, 0]
        self.zoom = 2
        self.map = Map(location=self.map_center, zoom_start=self.zoom, control_scale=True)
        self.markers_by_topic = {topic: [] for topic in self.topics}
        self.temp_html = os.path.join(tempfile.gettempdir(), "map.html")

        self.view = QWebEngineView()
        self.update_map()

        layout = QVBoxLayout()
        hlayout = QHBoxLayout()

        for topic in self.topics:
            box = QCheckBox(topic)
            box.setChecked(True)
            box.stateChanged.connect(lambda _, t=topic: self.toggle_topic(t))
            hlayout.addWidget(box)

        clear_btn = QPushButton("Clear")
        clear_btn.clicked.connect(self.clear_map)
        hlayout.addWidget(clear_btn)

        layout.addLayout(hlayout)
        layout.addWidget(self.view)
        self.setLayout(layout)

        self.ros = ROSBridge(self.topics)
        self.ros.new_fix.connect(self.add_marker)
        rospy.init_node("navsat_map_viewer", anonymous=True)

        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.update_map)
        self.refresh_timer.start(2000)  # update every 2 seconds

    def toggle_topic(self, topic):
        if topic in self.active_topics:
            self.active_topics.remove(topic)
        else:
            self.active_topics.add(topic)
        self.update_map()

    def clear_map(self):
        self.markers_by_topic = {topic: [] for topic in self.topics}
        self.map = Map(location=self.map_center, zoom_start=self.zoom, control_scale=True)
        self.update_map()

    def add_marker(self, topic, lat, lon):
        if topic not in self.active_topics:
            return
        self.markers_by_topic[topic].append((lat, lon))

    def update_map(self):
        self.map = Map(location=self.map_center, zoom_start=self.zoom, control_scale=True)
        for topic, markers in self.markers_by_topic.items():
            if topic in self.active_topics:
                for lat, lon in markers:
                    Marker(location=[lat, lon], popup=topic).add_to(self.map)
        self.map.save(self.temp_html)
        self.view.load(f"file://{self.temp_html}")
