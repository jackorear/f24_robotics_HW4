import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jack/f24_robotics_HW4/webots_apriltags/install/webots_apriltags'
