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

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703')
DEFAULT_HEIGHT = 0.5
DURATION = 10
deck_attached_event = Event()
logging.basicConfig(level=logging.ERROR)
_time = []

log_vars = {
    "controller.cmd_roll": {"type": "float", "unit": "cmd", "data": []},
    "controller.cmd_pitch": {"type": "float", "unit": "cmd", "data": []},
    "controller.cmd_yaw": {"type": "float", "unit": "cmd", "data": []},
}

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


def plot_metrics():
    timestamps = [d["timestamp"] for d in gyro_data]
    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(timestamps, [d["roll"] for d in gyro_data], label='Roll')
    plt.plot(timestamps, [d["pitch"] for d in gyro_data], label='Pitch')
    plt.plot(timestamps, [d["yaw"] for d in gyro_data], label='Yaw')
    plt.legend()
    plt.title("Gyroscope Data")

    plt.subplot(3, 1, 2)
    plt.plot(timestamps, [d["acc_x"] for d in accel_data], label='Acc X')
    plt.plot(timestamps, [d["acc_y"] for d in accel_data], label='Acc Y')
    plt.plot(timestamps, [d["acc_z"] for d in accel_data], label='Acc Z')
    plt.legend()
    plt.title("Accelerometer Data")

    plt.subplot(3, 1, 3)
    plt.plot(timestamps, [d["m1"] for d in motor_output_data], label='Motor 1')
    plt.plot(timestamps, [d["m2"] for d in motor_output_data], label='Motor 2')
    plt.plot(timestamps, [d["m3"] for d in motor_output_data], label='Motor 3')
    plt.plot(timestamps, [d["m4"] for d in motor_output_data], label='Motor 4')
    plt.legend()
    plt.title("Motor Output Data")

    plt.tight_layout()
    plt.show()


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

        plot_metrics()
