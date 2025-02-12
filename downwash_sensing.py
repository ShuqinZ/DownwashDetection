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

log_vars = {
    "Gyro_data": {
        "timestamp": {"type": "float", "unit": "s", "data": []},
        "stateEstimate.roll": {"type": "float", "unit": "deg", "data": []},
        "stateEstimate.pitch": {"type": "float", "unit": "deg", "data": []},
        "stateEstimate.yaw": {"type": "float", "unit": "deg", "data": []}
    },

    "Accel_data": {
        "timestamp": {"type": "float", "unit": "s", "data": []},
        "stateEstimate.ax": {"type": "float", "unit": "Gs", "data": []},
        "stateEstimate.ay": {"type": "float", "unit": "Gs", "data": []},
        "stateEstimate.az": {"type": "float", "unit": "Gs", "data": []},
    },

    "Motor_data": {
        "timestamp": {"type": "float", "unit": "s", "data": []},
        "motor.m1": {"type": "float", "unit": "UINT16", "data": []},
        "motor.m2": {"type": "float", "unit": "UINT16", "data": []},
        "motor.m3": {"type": "float", "unit": "UINT16", "data": []},
        "motor.m4": {"type": "float", "unit": "UINT16", "data": []},
    },
}


def log_callback(timestamp, data, logconf):
    # print(timestamp, data)

    log = log_vars[logconf.name + "_data"]

    log["timestamp"]["data"].append(timestamp)

    for par in log_vars.keys()[1:]:
        log[par]["data"].append(data[par])


def plot_metrics(file=""):
    if not os.path.exists('metrics'):
        os.makedirs('metrics', exist_ok=True)

    if file:
        with open(file) as f:
            json_data = json.load(f)
            _time = json_data["time"]
            log_vars = json_data["params"]

    filename = f"{datetime.datetime.now():%Y_%m_%d_%H_%M_%S}"

    fig, axis = plt.subplots(nrows=len(log_vars.keys()), ncols=1)

    for component, ax in zip(log_vars.keys(), axis):
        log = log_vars[component]

        time_axis = (np.array(log["timestamp"]["data"]) - log["timestamp"]["data"][0]) / 1000
        for par in log.keys()[1:]:
            ax.plot(time_axis, np.array(log_vars[par]["data"]), label=f"{par} ({log[par]['unit']})")
        ax.set_xlabel('Time (s)')
        ax.legend()
        ax.set_title(component)

    if not file:
        plt.savefig(f'metrics/{filename}.png', dpi=300)
        with open(f'metrics/{filename}.json', 'w') as f:
            json.dump(log_vars, f, indent=4)
    else:
        image_name = "".join(file.split('.')[:-1])
        plt.savefig(f'{image_name}.png', dpi=300)


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
        gyro_logconf.data_received_cb.add_callback(log_callback)
        gyro_logconf.start()

        accel_logconf = LogConfig(name='Accel', period_in_ms=10)
        accel_logconf.add_variable('stateEstimate.ax', 'float')
        accel_logconf.add_variable('stateEstimate.ay', 'float')
        accel_logconf.add_variable('stateEstimate.az', 'float')
        scf.cf.log.add_config(accel_logconf)
        accel_logconf.data_received_cb.add_callback(log_callback)
        accel_logconf.start()

        motor_logconf = LogConfig(name='Motor', period_in_ms=10)
        motor_logconf.add_variable('motor.m1', 'uint16_t')
        motor_logconf.add_variable('motor.m2', 'uint16_t')
        motor_logconf.add_variable('motor.m3', 'uint16_t')
        motor_logconf.add_variable('motor.m4', 'uint16_t')
        scf.cf.log.add_config(motor_logconf)
        motor_logconf.data_received_cb.add_callback(log_callback)
        motor_logconf.start()

        time.sleep(DURATION)

        gyro_logconf.stop()
        accel_logconf.stop()
        motor_logconf.stop()

        plot_metrics()
