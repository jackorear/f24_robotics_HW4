import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/areed/aprilTags/f24_robotics_HW4/install/webots_apriltags'
