import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/labpc1/dflqp_unicycle_ws/install/controllers'
