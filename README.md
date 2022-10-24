# image_matching
Image matching using sift
All scripts located at image_processing/src/image_processing
anti_spoof.py - spoof detection
bag_recorder.py - rosbag automatic record
command_logger.py - command logging to .log
csi_camera.py - driver for csi camera
geodetic_conv.py - geo convertions class
gps_noiser.py - noise gps for anti_spoof test
image_logger.py - csi camera video recorder during flight
image_processing.py - class for basic image processing
logger_plata.py - csv logger for imu gps and baro
logger.py - main logger for video localization
match_finder.py - main class for image matching
param_publisher.py - publish params from params.yaml while startup
photo_publisher.py - video pub for offline tests
plata_logger_for_analize.py - synchronize logging
position_finder.py - main file for position finding
simple_kalman.py - simple kalman filter for video localization
utils.py - utils functions
App launchs by the .launch file launch/find_pose.launch

