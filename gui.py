import sys
import os
import cv2
import time
from datetime import datetime, timedelta
import numpy as np
import pyrealsense2 as rs

from PySide6.QtCore import QThread, Signal, Qt, Slot, QTimer
from PySide6.QtWidgets import (
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
    QLabel,
    QHBoxLayout
    )
from PySide6.QtGui import QImage, QPixmap, QFont


## Class to engage RealSense camera ##
## ================================ ##

class RealSenseCamera(QThread):
    # signal to send the frame and stats back to the GUI for display
    image_data = Signal(np.ndarray)
    stats_data = Signal(int, str)
    elapsed_time = Signal(float)
    
    def __init__(
            self,
            serial_number=None,
            folder_path='.',
            frames_per_file=1000,
            width=640,
            height=480,
            fps=30,
            codec='XVID',
            exposure=5000,
            gain=50,
            laser_power=150,
            enable_ttl=True
            ):
        '''
        Initialize RealSense camera, define recording
        parameters, and enable infrared stream.

        Parameters
        ==========
        serial_number : str
            Serial number of the desired camera to enable. If None,
            default camera will be enabled. Default is None.
        folder_path : str
            Directory where the files will be stored. A folder with
            the date and time will be created there. Default is '.'.
        frames_per_file : int
            Number of frames that each avi file will store. Default is 1000.
        width, height : int
            Width and height of the FOV of the camera. Defaults are 640 and
            480, respectively.
        fps : int
            Desired frame rate of the camera. Default is 30.
        codec : str
            Which codec to use to save avi files. Default is 'XVID'.
        exposure : int
            Exposure of the camera stream. Default is 5000.
        gain : int
            Digital gain of the camera stream. Default is 50.
        laser_power : int
            Power of the infrared laser in the camera. Default is 150.
        enable_ttl : bool
            Whether or not to use output trigger on every frame. Default is True.
        '''
        super().__init__()
        self.recording = False
        self.running = True
        
        # initialize variables
        self.writer = None
        self.file_counter = 0
        self.frame_counter = 0
        self.total_frames = 0
        self.recording_start_time = None
        self.serial_number = serial_number
        self.folder_path = folder_path
        self.frames_per_file = frames_per_file
        self.width = width
        self.height = height
        self.fps = fps
        self.codec = codec
        self.exposure = exposure
        self.gain = gain
        self.laser_power = laser_power
        self.enable_ttl = enable_ttl
        
        # set up realsense stream
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        if self.serial_number is not None:
            self.config.enable_device(serial_number)
        self.config.enable_stream(
            rs.stream.infrared,
            1,
            width,
            height,
            rs.format.y8,
            fps
            )

    def run(self):
        '''
        Start camera and while recording is enabled (triggered
        externally), capture, display, and save frames. Display
        time elapsed as well.
        '''
        profile = self.pipeline.start(self.config)
        device = profile.get_device()
        # note: depth and infrared are on the same sensor
        self.depth_sensor = device.first_depth_sensor()

        # manual exposure
        self.depth_sensor.set_option(rs.option.enable_auto_exposure, 0)
        self.depth_sensor.set_option(rs.option.exposure, self.exposure)
        self.depth_sensor.set_option(rs.option.gain, self.gain)
        self.depth_sensor.set_option(rs.option.laser_power, self.laser_power)

        try:
            while self.running:
                frames = self.pipeline.wait_for_frames()
                ir_frame = frames.get_infrared_frame()
                if not ir_frame:
                    continue
                frame_np = np.asanyarray(ir_frame.get_data())
                self.image_data.emit(frame_np) # display frame

                if self.recording:
                    self.write_frame(frame_np)

                    elapsed = time.time() - self.recording_start_time
                    time_str = str(timedelta(seconds=int(elapsed)))
                    self.stats_data.emit(self.total_frames, time_str)
                    self.elapsed_time.emit(elapsed)
                else:
                    self.stats_data.emit(0, "00:00:00")

        finally:
            self.stop_recording()
            self.pipeline.stop()

    def write_frame(self, frame):
        '''
        Write current frame to the currently open AVI file and advance
        frame counter. If the file is full, rotate to next file.
        '''
        if self.writer is None or self.frame_counter >= self.frames_per_file:
            self.rotate_file()
            
        self.writer.write(frame)
        self.frame_counter += 1
        self.total_frames += 1

    def rotate_file(self):
        '''
        Release AVI file currently being written and make new file.
        '''
        if self.writer:
            self.writer.release()
            self.file_counter += 1
            self.frame_counter = 0
        
        # create new compressed AVI file
        filename = os.path.join(self.folder_path, f"{self.file_counter}.avi")
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        self.writer = cv2.VideoWriter(
            filename,
            fourcc,
            self.fps,
            (self.width, self.height),
            isColor=False
            )
        print(f"Recording to: {filename}")

    def start_recording(self):
        '''
        Initialize frame and file counters, note current time,
        make file directory if it doesn't exist, and switch recording
        flag to True. Enable TTL.
        '''
        if not self.recording:
            self.file_counter = 0
            self.frame_counter = 0
            self.total_frames = 0
            self.recording_start_time = time.time()

            # define folder and create if doesn't exist
            current_time = str(datetime.now().strftime("%Y_%m_%d-%H_%M_%S"))
            self.folder_path = os.path.join(self.folder_path, current_time)
            if not os.path.exists(self.folder_path):
                os.makedirs(self.folder_path)

            # turn on ttl
            if self.enable_ttl:
                if self.depth_sensor.supports(rs.option.output_trigger_enabled):
                    self.depth_sensor.set_option(rs.option.output_trigger_enabled, 1)

            self.recording = True

    def stop_recording(self):
        '''
        Switch off recording and release AVI file currently being
        written. Disable TTL.
        '''
        self.recording = False

        # turn off TTL
        if self.enable_ttl:
            if self.depth_sensor.supports(rs.option.output_trigger_enabled):
                self.depth_sensor.set_option(rs.option.output_trigger_enabled, 0)

        if self.writer:
            self.writer.release()
            self.writer = None
            print('Recording stopped.')

    def stop_camera(self):
        '''
        Stop running camera (used when GUI closes).
        '''
        self.running = False
        self.wait()


## Class to display and operate recording GUI ##
## ========================================== ##

class RealSenseGUI(QMainWindow):
    def __init__(
            self,
            recording_length=None,
            serial_number=None,
            folder_path='.',
            frames_per_file=1000,
            width=640,
            height=480,
            fps=30,
            codec='XVID',
            exposure=5000,
            gain=50,
            laser_power=150,
            enable_ttl=True
            ):
        '''
        Initialize GUI elements, with spaces for the video
        feed, time elapsed, a recording indicator light, and
        buttons to begin and end recording. Also initialize
        camera object.

        Parameters
        ==========
        recording_length : int
            Time in seconds to make the recording. If 0 or None, will be converted to np.inf and
            the recording will run until the recording is manually stopped. Default is None.
        ### The rest of the parameters are kwargs fed to the RealSenseCamera class ###
        '''
        super().__init__()
        self.setWindowTitle("RealSense IR Recorder")
        self.setFixedSize(700, 600)

        # set variables
        self.serial_number = serial_number
        self.folder_path = folder_path
        self.recording_length = np.inf if (recording_length==0)|(recording_length is None) else recording_length
        self.frames_per_file = frames_per_file
        self.width = width
        self.height = height
        self.fps = fps
        self.codec = codec
        self.exposure = exposure
        self.gain = gain
        self.laser_power = laser_power
        self.enable_ttl = enable_ttl

        # ---------------------------------------------
        # UI layout initialization
        self.central_widget = QWidget()
        self.layout = QVBoxLayout(self.central_widget)

        # display stats
        self.stats_layout = QHBoxLayout()
        self.time_label = QLabel("Duration: 00:00:00")
        self.count_label = QLabel("Frames: 0")
        stat_font = QFont("Arial", 14, QFont.Bold)
        self.time_label.setFont(stat_font)
        self.count_label.setFont(stat_font)
        self.time_label.setStyleSheet("color: #2ecc71;")
        self.stats_layout.addWidget(self.time_label)
        self.stats_layout.addStretch()
        self.stats_layout.addWidget(self.count_label)
        self.layout.addLayout(self.stats_layout)

        # display video label
        self.video_label = QLabel("Camera Feed")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(self.width, self.height)
        self.video_label.setStyleSheet(
            "background-color: black; border: 2px solid gray;"
            )
        self.layout.addWidget(self.video_label)

        # create and connect buttons
        self.btn_layout = QHBoxLayout()
        self.record_btn = QPushButton("Record")
        self.stop_btn = QPushButton("Stop Recording")
        self.stop_btn.setEnabled(False)
        
        self.btn_layout.addWidget(self.record_btn)
        self.btn_layout.addWidget(self.stop_btn)
        self.layout.addLayout(self.btn_layout)
        
        self.setCentralWidget(self.central_widget)
        self.record_btn.clicked.connect(self.on_record_clicked)
        self.stop_btn.clicked.connect(self.on_stop_clicked)

        # add blinking light when recording
        self.blink_timer = QTimer()
        self.blink_timer.setInterval(500) # every 500ms
        self.blink_timer.timeout.connect(self.toggle_light_blink)
        self.blink_state = False
        self.rec_indicator = QLabel()
        self.rec_indicator.setFixedSize(20, 20)
        self.rec_indicator.setStyleSheet(
            "background-color: #444; border-radius: 10px; border: 1px solid #222;"
            )
        self.stats_layout.addWidget(self.rec_indicator)

        # ---------------------------------------------

        # start camera thread
        self.camera = RealSenseCamera(
            serial_number=self.serial_number,
            folder_path=self.folder_path,
            frames_per_file=self.frames_per_file,
            width=self.width,
            height=self.height,
            fps=self.fps,
            codec=self.codec,
            exposure=self.exposure,
            gain=self.gain,
            laser_power=self.laser_power,
            enable_ttl=self.enable_ttl
            )
        self.camera.image_data.connect(self.update_image)
        self.camera.stats_data.connect(self.update_stats)
        self.camera.elapsed_time.connect(self.check_recording_length)
        self.camera.start()

    @Slot(np.ndarray)
    def update_image(self, frame):
        '''
        Update displayed image.
        '''
        height, width = frame.shape
        bytes_per_line = width
        q_img = QImage(frame.data, width, height, bytes_per_line, QImage.Format_Grayscale8)
        self.video_label.setPixmap(QPixmap.fromImage(q_img))
    
    @Slot(int, str)
    def update_stats(self, count, elapsed_time):
        '''
        Update time elapsed in frames and time.
        '''
        self.count_label.setText(f"Frames: {count}")
        self.time_label.setText(f"Duration: {elapsed_time}")

    def toggle_light_blink(self):
        '''
        Turn on blinking red light when recording; turn back to
        grey when not recording.
        '''
        if self.blink_state:
            self.rec_indicator.setStyleSheet(
                "background-color: #444; border-radius: 10px; border: 1px solid #222;"
                )
        else:
            self.rec_indicator.setStyleSheet(
                "background-color: #ff0000; border-radius: 10px; border: 1px solid #a00;"
                )
        self.blink_state = not self.blink_state

    def on_record_clicked(self):
        '''
        Disable record button when clicked, begin
        timer, start blinking light, and start recording.
        '''
        self.record_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

        self.blink_timer.start()
        self.rec_indicator.setStyleSheet(
            "background-color: #ff0000; border-radius: 10px; border: 1px solid #a00;"
            )
        self.blink_state = True

        self.camera.start_recording()

    def on_stop_clicked(self):
        '''
        Disable stop recording button when clicked, stop
        timer, stop blinking light, and stop recording.
        '''
        self.camera.stop_recording()

        self.record_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

        self.blink_timer.stop()
        self.rec_indicator.setStyleSheet(
            "background-color: #444; border-radius: 10px; border: 1px solid #222;"
            )
        self.blink_state = False


    @Slot(float)
    def check_recording_length(self, elapsed):
        '''
        When recording length has completed (if provided), end recording.
        '''
        if elapsed >= self.recording_length:
            self.on_stop_clicked()

    def closeEvent(self, event):
        '''
        When GUI is closed, stop streaming camera.
        '''
        self.camera.stop_camera()
        super().closeEvent(event)
