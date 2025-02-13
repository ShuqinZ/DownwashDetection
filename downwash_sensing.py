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
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
import matplotlib.pyplot as plt
import numpy as np
import argparse

URI_LIST = {
    "LowerFLS": 'radio://0/80/2M/E7E7E7E703',
    "UpperFLS": 'radio://0/80/2M/E7E7E7E703'
}

UPPER_WAIT_TIME = 0
DURATION = 20
deck_attached_event = Event()
logging.basicConfig(level=logging.ERROR)

log_vars = {
    "Gyro_data": {
        "timestamp": [],
        "values": {
            "stateEstimate.roll": {"type": "float", "unit": "deg", "data": []},
            "stateEstimate.pitch": {"type": "float", "unit": "deg", "data": []},
            "stateEstimate.yaw": {"type": "float", "unit": "deg", "data": []}
        }
    },

    "Accel_data": {
        "timestamp": [],
        "values": {
            "stateEstimate.ax": {"type": "float", "unit": "Gs", "data": []},
            "stateEstimate.ay": {"type": "float", "unit": "Gs", "data": []},
            "stateEstimate.az": {"type": "float", "unit": "Gs", "data": []},
        }
    },

    "Motor_data": {
        "timestamp": [],
        "values": {
            "motor.m1": {"type": "float", "unit": "UINT16", "data": []},
            "motor.m2": {"type": "float", "unit": "UINT16", "data": []},
            "motor.m3": {"type": "float", "unit": "UINT16", "data": []},
            "motor.m4": {"type": "float", "unit": "UINT16", "data": []},
        }
    },
}


def select_uri(func):
    def wrapper(uri, *args, **kwargs):
        global URI_LIST
        if uri == URI_LIST["LowerFLS"]:
            func(*args, **kwargs)

    return wrapper


@select_uri
def log_callback(timestamp, data, logconf):
    # print(timestamp, data)

    log_data = log_vars[logconf.name + "_data"]["values"]

    log_vars[logconf.name + "_data"]["timestamp"].append(timestamp)

    for par in log_data.keys():
        log_data[par]["data"].append(data[par])


def plot_metrics(file=""):
    global log_vars
    if not os.path.exists('metrics'):
        os.makedirs('metrics', exist_ok=True)

    if file:
        with open(file) as f:
            json_data = json.load(f)
            _time = json_data["time"]
            log_vars = json_data["params"]

    filename = f"{datetime.datetime.now():%Y_%m_%d_%H_%M_%S}"

    fig, axis = plt.subplots(nrows=len(log_vars.keys()), ncols=1, figsize=(12, 8))

    for component, ax in zip(log_vars.keys(), axis):
        log_data = log_vars[component]["values"]

        timeline = log_vars[component]["timestamp"]
        time_axis = (np.array(timeline) - timeline[0]) / 1000

        for par in log_data.keys():
            line_name = "".join(par.split('.')[-1])
            ax.plot(time_axis, np.array(log_data[par]["data"]), label=f"{line_name} ({log_data[par]['unit']})")
        ax.set_xlabel('Time (s)')
        ax.legend(loc="upper left")
        ax.set_title(component)

    plt.tight_layout(h_pad=2)

    if not file:
        plt.savefig(f'metrics/{filename}.png', dpi=300)
        with open(f'metrics/{filename}.json', 'w') as f:
            json.dump(log_vars, f, indent=4)
    else:
        image_name = "".join(file.split('.')[:-1])
        plt.savefig(f'{image_name}.png', dpi=300)


@select_uri
def start_logger(scf):
    gyro_logconf = LogConfig(name='Gyro', period_in_ms=10)
    gyro_logconf.add_variable('stateEstimate.roll', 'float')
    gyro_logconf.add_variable('stateEstimate.pitch', 'float')
    gyro_logconf.add_variable('stateEstimate.yaw', 'float')
    scf.cf.log.add_config(gyro_logconf)
    gyro_logconf.data_received_cb.add_callback(
        lambda timestamp, data, logconf: log_callback(scf.cf.link_uri, timestamp, data, logconf)
    )

    accel_logconf = LogConfig(name='Accel', period_in_ms=10)
    accel_logconf.add_variable('stateEstimate.ax', 'float')
    accel_logconf.add_variable('stateEstimate.ay', 'float')
    accel_logconf.add_variable('stateEstimate.az', 'float')
    scf.cf.log.add_config(accel_logconf)
    accel_logconf.data_received_cb.add_callback(
        lambda timestamp, data, logconf: log_callback(scf.cf.link_uri, timestamp, data, logconf)
    )

    motor_logconf = LogConfig(name='Motor', period_in_ms=10)
    motor_logconf.add_variable('motor.m1', 'uint16_t')
    motor_logconf.add_variable('motor.m2', 'uint16_t')
    motor_logconf.add_variable('motor.m3', 'uint16_t')
    motor_logconf.add_variable('motor.m4', 'uint16_t')
    scf.cf.log.add_config(motor_logconf)
    motor_logconf.data_received_cb.add_callback(
        lambda timestamp, data, logconf: log_callback(scf.cf.link_uri, timestamp, data, logconf)
    )

    # Start Logger
    gyro_logconf.start()
    accel_logconf.start()
    motor_logconf.start()

    # return [gyro_logconf, accel_logconf, motor_logconf]


# @select_uri
# def stop_logger(scf, loggers):
#     for logger in loggers:
#         logger.stop()

def async_flight(scf, upper_wait_time, duration):
        start_time = time.time()
        end_time = start_time + duration

        scf.cf.commander.send_setpoint(0, 0, 0, 0)

        roll = 0.0
        pitch = 0.0
        yawrate = 0
        thrust = 50000

        while time.time() < end_time:
            if scf.__dict__['_link_uri'] == URI_LIST["LowerFLS"]:
                # thrust_step = 200
                # thrust_mult = 1

                scf.cf.commander.send_setpoint(roll, pitch, yawrate, thrust)

            elif scf.__dict__['_link_uri'] == URI_LIST["UpperFLS"]:

                if time.time() < start_time + upper_wait_time:
                    continue
                scf.cf.commander.send_setpoint(roll, pitch, yawrate, thrust)

        scf.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.01)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(URI_LIST, factory=factory) as swarm:

        swarm.reset_estimators()
        time.sleep(1.0)

        swarm.parallel_safe(start_logger)
        time.sleep(0.1)

        swarm.parallel_safe(async_flight, UPPER_WAIT_TIME, DURATION)

        plot_metrics()
