import sys
from PySide6.QtWidgets import QApplication
from gui import RealSenseGUI

recording_params = {
    'serial_number'   : '213622074070', # leave as None to load the first available one
    'folder_path'     : './data/',
    'recording_length': 10, # in seconds, leave 0 if you want recording indefinitely
    'frames_per_file' : 1000,
    'width'       : 640,
    'height'      : 480,
    'fps'         : 30,
    'codec'       : 'XVID',
    'exposure'    : 2500,
    'gain'        : 50,
    'laser_power' : 200,
    'enable_ttl'  : True,
    
    # tracking parameters
    'use_tracking'    : True,
    'ref_num_frames'  : 30,
    'tracking_method' : 'dark',
    'use_window'      : True,
    'window_size'     : 100,
    'window_weight'   : 0.90,
    'loc_thresh'      : 99.5,
    'ksize'           : 5,
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