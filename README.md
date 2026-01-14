## GUI to record infrared stream from RealSense D435f camera and save into avi files

This repository defines a `RealSenseCamera` class which engages a RealSense camera to grab <br>
frames and send output TTL signals on every frame. It also defines a `RealSenseGUI` class which <br>
creates a GUI written using `PySide6` to stream the video and to click record/stop recording.
***
To use the GUI:
1. Navigate to this folder after cloning this repo: `cd /path/to/folder/RealSense_Recorder`.
2. Create conda environment: `conda env create -f your_env_file.yaml`.
    - Replace `your_env_file` with `env_mac` or `env_windows` depending on your OS.
3. Activate env: `source activate realsense`.
4. In the `record_infrared.py` file, fill out the `recording_params` as desired.
    - Refer to the documentation in `gui.py` for details.
5. Run `python record_infrared.py` to boot up the GUI. Some notes:
    - _On Mac, you may need to run the above command using `sudo`._
    - _Sometimes, on boot you might get an error where the camera could not<br>connect. Try closing the GUI and running again._
    - _If recording from multiple streams simultaneously, it is recommended to<br>make multiple copies of the `record_infrared.py` file, rename the files to<br>reflect the streams, and run them in their own separate terminal windows._
    - _It is also recommended to change the `folder_path` in each py file such that<br>the separate streams are saved to different folders._
6. Click 'Record' to start and 'Stop Recording' to stop.
