class Config:
    # URI_LIST = ['radio://0/80/2M/E7E7E7E702', 'radio://0/80/2M/E7E7E7E704']
    URI_LIST = ['radio://0/80/2M/E7E7E7E702']
    ITERATIONS = 1
    DURATION = 120
    GAP = 0
    WAIT_TIME = 0
    THRUST = 35000
    ROLL = 0
    PITCH = 0
    YAWRATE = 0
    DELTA_X = 0
    DELTA_Z = 0
    DELTA_YAW = 0
    # CONFIG = f"x{DELTA_X}_z{DELTA_Z}_yaw{DELTA_YAW}_TH{THRUST}_R{ROLL}_P{PITCH}_YR{YAWRATE}"
    CONFIG = f"Stable"
    DEBUG = [0, 0]  # 0 for actual flight, 1 for print all command, 2 for print command changes, 3 for do nothing
