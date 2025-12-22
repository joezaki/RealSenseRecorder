## GUI to record infrared stream from RealSense D435f camera and save into avi files

This repository defines a `RealSenseCamera` class which engages a RealSense camera to grab <br>
frames and send output TTL signals on every frame. It also defines a `RealSenseGUI` class which <br>
creates a GUI written using `PySide6` to stream the video and to click record/stop recording.
***
To use the GUI:
- Navigate to this folder after cloning this repo: `cd /path/to/folder/RealSense_Recorder`
- Create conda environment: `conda env create -f environment.yaml`
- Activate env: `source activate realsense`
- In the `record_infrared.py` file, fill out the `recording_params` as desired.
- Run `python record_infrared.py` to boot up the GUI.
- Click 'Record' to start and 'Stop Recording' to stop.
