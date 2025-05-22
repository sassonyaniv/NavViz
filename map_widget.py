from PyQt5.QtWidgets import QWidget, QVBoxLayout, QCheckBox, QPushButton, QHBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel
from PyQt5.QtCore import QUrl, QObject, pyqtSlot
import os
import rospy
from ros_bridge import ROSBridge

class MapInterface(QObject):
    def __init__(self):
        super().__init__()
        self.view = None

    @pyqtSlot(str, float, float)
    def sendFix(self, topic, lat, lon):
        if self.view:
            self.view.page().runJavaScript(f"addMarker('{topic}', {lat}, {lon})")

    @pyqtSlot()
    def clearMap(self):
        if self.view:
            self.view.page().runJavaScript("clearMap()")

class MapWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NavSatFix Viewer")
        self.topics = ["/gps/fix", "/other/fix"]
        self.active_topics = set(self.topics)

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

        self.view = QWebEngineView()
        map_file = os.path.join(os.path.dirname(__file__), "map_logic.html")
        self.view.load(QUrl.fromLocalFile(map_file))
        layout.addWidget(self.view)
        self.setLayout(layout)

        self.bridge = ROSBridge(self.topics)
        self.bridge.new_fix.connect(self.on_new_fix)

        self.channel = QWebChannel()
        self.js_interface = MapInterface()
        self.js_interface.view = self.view
        self.channel.registerObject("pyjs", self.js_interface)
        self.view.page().setWebChannel(self.channel)

        rospy.init_node("navsat_map_viewer", anonymous=True)

    def toggle_topic(self, topic):
        if topic in self.active_topics:
            self.active_topics.remove(topic)
        else:
            self.active_topics.add(topic)

    def clear_map(self):
        self.js_interface.clearMap()

    def on_new_fix(self, topic, lat, lon):
        if topic in self.active_topics:
            self.js_interface.sendFix(topic, lat, lon)
