class Config:
    URI_LIST = ['radio://0/80/2M/E7E7E7E702', 'radio://0/80/2M/E7E7E7E704']
    ITERATIONS = 6
    DURATION = 10
    GAP = 5
    WAIT_TIME = 5
    THRUST = 35000
    ROLL = 5
    PITCH = 0
    YAWRATE = 0
    DELTA_X = 0
    DELTA_Z = 24
    DELTA_YAW = 0
    CONFIG = f"x{DELTA_X}_z{DELTA_Z}_yaw{DELTA_YAW}_TH{THRUST}_R{ROLL}_P{PITCH}_YR{YAWRATE}"
    DEBUG = [0, 0]  # 0 for actual flight, 1 for print all command, 2 for print command changes, 3 for do nothing
