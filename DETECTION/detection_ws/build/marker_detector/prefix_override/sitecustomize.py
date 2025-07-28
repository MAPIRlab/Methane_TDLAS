import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aromsan/TDLAS-Perception-Detection/detection_ws/install/marker_detector'
