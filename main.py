# --- main.py ---
from PyQt5.QtWidgets import QApplication
from map_widget import MapWindow
import sys

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MapWindow()
    window.show()
    sys.exit(app.exec_())
