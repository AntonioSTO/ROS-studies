import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/antonio/Desktop/ros2_ws/src/install/turtle_control_ant'
