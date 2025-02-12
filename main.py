# sample usage for plotting
# python3 main.py -p [path_to_json_file]

import datetime
import json
import logging
import os
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper
import matplotlib.pyplot as plt
import numpy as np
import argparse

URI = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E701')
DEFAULT_HEIGHT = 0.5
DURATION = 10
deck_attached_event = Event()
logging.basicConfig(level=logging.ERROR)
_time = []

gyro_data = []
accel_data = []
motor_output_data = []

def log_gyro_callback(timestamp, data, logconf):
    gyro_data.append({
        "timestamp": timestamp,
        "roll": data["stateEstimate.roll"],
        "pitch": data["stateEstimate.pitch"],
        "yaw": data["stateEstimate.yaw"],
    })

def log_accel_callback(timestamp, data, logconf):
    accel_data.append({
        "timestamp": timestamp,
        "acc_x": data["acc.x"],
        "acc_y": data["acc.y"],
        "acc_z": data["acc.z"],
    })

def log_motor_callback(timestamp, data, logconf):
    motor_output_data.append({
        "timestamp": timestamp,
        "m1": data["motor.m1"],
        "m2": data["motor.m2"],
        "m3": data["motor.m3"],
        "m4": data["motor.m4"],
    })

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        gyro_logconf = LogConfig(name='Gyro', period_in_ms=10)
        gyro_logconf.add_variable('stateEstimate.roll', 'float')
        gyro_logconf.add_variable('stateEstimate.pitch', 'float')
        gyro_logconf.add_variable('stateEstimate.yaw', 'float')
        scf.cf.log.add_config(gyro_logconf)
        gyro_logconf.data_received_cb.add_callback(log_gyro_callback)
        gyro_logconf.start()

        accel_logconf = LogConfig(name='Accel', period_in_ms=10)
        accel_logconf.add_variable('acc.x', 'float')
        accel_logconf.add_variable('acc.y', 'float')
        accel_logconf.add_variable('acc.z', 'float')
        scf.cf.log.add_config(accel_logconf)
        accel_logconf.data_received_cb.add_callback(log_accel_callback)
        accel_logconf.start()

        motor_logconf = LogConfig(name='Motor', period_in_ms=10)
        motor_logconf.add_variable('motor.m1', 'float')
        motor_logconf.add_variable('motor.m2', 'float')
        motor_logconf.add_variable('motor.m3', 'float')
        motor_logconf.add_variable('motor.m4', 'float')
        scf.cf.log.add_config(motor_logconf)
        motor_logconf.data_received_cb.add_callback(log_motor_callback)
        motor_logconf.start()

        time.sleep(DURATION)

        gyro_logconf.stop()
        accel_logconf.stop()
        motor_logconf.stop()

        print("Gyro Data:", gyro_data)
        print("Accelerometer Data:", accel_data)
        print("Motor Output Data:", motor_output_data)
