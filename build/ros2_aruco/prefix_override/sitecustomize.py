import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/oumuika/Documents/mirs2403/src/ros2_aruco/install/ros2_aruco'
