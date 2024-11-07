import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/neolux/Desktop/neo_qrcode/install/qrc_skandier'
