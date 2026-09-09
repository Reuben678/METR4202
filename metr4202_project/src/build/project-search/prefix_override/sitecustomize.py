import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mjc/METR4202/METR4202/metr4202_project/src/install/project-search'
