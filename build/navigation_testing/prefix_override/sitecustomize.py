import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/filipe/thesis/ros2_selfdriving/install/navigation_testing'
