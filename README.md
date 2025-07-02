Steps to start our system:
1. roscore
2. roslaunch usb_cam usb_cam-test.launch (we connect robot to camera by USB)
3. roslaunch opencv_apps face_recognition.launch image:=/usb_cam/image_raw (could adjust .launch file to train) 
4. rosrun robot_interaction RobotInteractionManager.py
5. rosrun speech_interaction google_sr.py