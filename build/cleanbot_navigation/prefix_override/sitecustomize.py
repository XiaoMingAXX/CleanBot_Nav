import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/xiaoming/桌面/MOON/Electronic/CleanBot_ws/install/cleanbot_navigation'
