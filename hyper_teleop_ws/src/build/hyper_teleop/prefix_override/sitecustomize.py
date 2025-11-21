import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/richi/hyper_teleop_ws/src/install/hyper_teleop'
