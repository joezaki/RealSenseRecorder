import sys
from PySide6.QtWidgets import QApplication
from gui import RealSenseGUI

recording_params = {
    'serial_number'   : '213622072819',
    'folder_path'     : './data',
    'frames_per_file' : 1000,
    'width'       : 640,
    'height'      : 480,
    'fps'         : 30,
    'codec'       : 'XVID',
    'exposure'    : 2500,
    'gain'        : 50,
    'laser_power' : 200,
    'enable_ttl'  : True,
}

## Run GUI ##
## ======= ##

if __name__ == "__main__":
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    gui = RealSenseGUI(**recording_params)
    gui.show()
    sys.exit(app.exec())