1. Start ROS in a separate console:
 roscore

2. Start camera:
rosrun camera1394 camera1394_node _guid:=000a4701110a4af8 _video_mode:=640x480_yuv411 _iso_speed:=800 _frame_rate:=30

or

rosrun camera1394 camera1394_node _guid:=000a4701110a4af8 _video_mode:=320x240_yuv422 _iso_speed:=800 _frame_rate:=60

3. View raw image:
rosrun image_view image_view image:=camera/image_raw

4. In case the camera need to be calibrated use: (instead of 2)
rosrun camera1394 camera1394_node _guid:=000a4701110a4af9 _video_mode:=640x480_yuv411 _iso_speed:=400 _frame_rate:=30 _camera_info_url:=file:///camera/calibrations/camera_640x480_right.yaml

And the start the calibration program using:
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/camera/image_raw camera:=/camera


