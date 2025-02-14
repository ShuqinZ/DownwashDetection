# sample usage for plotting
# python3 main.py -p [path_to_json_file]

import datetime
import json
import logging
import os
import sys
import threading
import time
from threading import Event, Lock

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.power_switch import PowerSwitch
import matplotlib.pyplot as plt
import numpy as np
import argparse

UPPER_WAIT_TIME = 10
DURATION = 15
PWM_SIGNAL = 35000
RECENT_TIME = 1  # sec
LOGRATE = 100  # Hz


PLOTTING = False

CONFIG = "x0_z6"

LOWERFLS_URI = 'radio://0/80/2M/E7E7E7E702'  # lower FLS
UPPERFLS_URI = 'radio://0/80/2M/E7E7E7E704'  # upper FLS

LOGGING_FLS = LOWERFLS_URI

URI_LIST = {
    LOWERFLS_URI,
    UPPERFLS_URI
}

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

log_vars_lock = Lock()

LOGGERS = []

def restart(uri):
    if isinstance(uri, list):
        for link in uri:
            PowerSwitch(link).stm_power_cycle()
    else:
        PowerSwitch(uri).stm_power_cycle()


def select_uri(func):
    def wrapper(uri, *args, **kwargs):
        global URI_LIST
        if uri == LOGGING_FLS:
            func(*args, **kwargs)

    return wrapper


@select_uri
def log_callback(timestamp, data, logconf):
    # print(timestamp, data)
    global log_vars

    with log_vars_lock:
        log_data = log_vars[logconf.name + "_data"]["values"]

        if not log_data or log_data is None:
            print(f"Missing log package: at time {timestamp}")
            return

        if timestamp is None or not timestamp:
            print(f"Missing timestamp: {log_vars[logconf.name + '_data']['timestamp']}")
            return

        log_vars[logconf.name + "_data"]["timestamp"].append(timestamp)

        for par in log_data.keys():
            value = data[par]
            if value is None:
                print(f"Missing Reading: {par} {value}")
                value = log_data[par]["data"][-1]
            log_data[par]["data"].append(value)


def plot_metrics(config, file=""):
    global log_vars
    if not os.path.exists('metrics'):
        os.makedirs('metrics', exist_ok=True)

    if not os.path.exists('metrics/' + config):
        os.makedirs('metrics/'+ config, exist_ok=True)


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
        plt.savefig(f'metrics/{config}/{filename}.png', dpi=300)
        with open(f'metrics/{config}/{filename}.json', 'w') as f:
            json.dump(log_vars, f, indent=4)
    else:
        image_name = "".join(file.split('.')[:-1])
        plt.savefig(f'metrics/{config}/{image_name}.png', dpi=300)


def plot_realtime(fps=50):
    """ Function to update real-time plots in a separate thread."""
    global log_vars, PLOTTING, RECENT_TIME, LOGRATE

    n = int(np.floor(RECENT_TIME * 1000 / LOGRATE))
    plt.ion()
    fig, axes = plt.subplots(nrows=len(log_vars.keys()), ncols=1, figsize=(12, 8))

    while PLOTTING:
        with log_vars_lock:
            for ax in axes:
                ax.clear()

            for component, ax in zip(log_vars.keys(), axes):
                log_data = log_vars[component]["values"]
                timeline = log_vars[component]["timestamp"]

                if len(timeline) > 0:
                    time_axis = (np.array(timeline) - timeline[0]) / 1000
                    start_idx = max(0, len(time_axis) - n)
                    time_axis = time_axis[start_idx:]

                    for par in log_data.keys():
                        data = np.array(log_data[par]["data"])[start_idx:]
                        ax.plot(time_axis, data, label=f"{par} ({log_data[par]['unit']})")

                    ax.set_xlabel('Time (s)')
                    ax.legend(loc="upper left")
                    ax.set_title(component)

        plt.tight_layout()
        plt.draw()
        plt.pause(1 / fps)

    plt.close(fig)


def stop_plot(plot_thread):
    global PLOTTING
    PLOTTING = False
    if plot_thread and plot_thread.is_alive():
        plot_thread.join()


def start_logger(scf):
    global LOWERFLS_URI, LOGRATE
    if scf.cf.link_uri != LOGGING_FLS:
        return

    gyro_logconf = LogConfig(name='Gyro', period_in_ms=1000/LOGRATE)
    gyro_logconf.add_variable('stateEstimate.roll', 'float')
    gyro_logconf.add_variable('stateEstimate.pitch', 'float')
    gyro_logconf.add_variable('stateEstimate.yaw', 'float')
    scf.cf.log.add_config(gyro_logconf)
    gyro_logconf.data_received_cb.add_callback(
        lambda timestamp, data, logconf: log_callback(scf.cf.link_uri, timestamp, data, logconf)
    )

    accel_logconf = LogConfig(name='Accel', period_in_ms=1000/LOGRATE)
    accel_logconf.add_variable('stateEstimate.ax', 'float')
    accel_logconf.add_variable('stateEstimate.ay', 'float')
    accel_logconf.add_variable('stateEstimate.az', 'float')
    scf.cf.log.add_config(accel_logconf)
    accel_logconf.data_received_cb.add_callback(
        lambda timestamp, data, logconf: log_callback(scf.cf.link_uri, timestamp, data, logconf)
    )

    motor_logconf = LogConfig(name='Motor', period_in_ms=1000/LOGRATE)
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

    LOGGERS.extend([gyro_logconf, accel_logconf, motor_logconf])


def stop_logger(loggers):
    for logger in loggers:
        logger.stop()


def async_flight(scf, start_time, wait_time, duration, pwm_signal):
    end_time = start_time + duration

    # print(start_time, wait_time, duration, pwm_signal, end_time)

    scf.cf.commander.send_setpoint(0, 0, 0, 0)

    roll = 0.0
    pitch = 0.0
    yawrate = 0

    while time.time() < end_time:
        if time.time() < start_time + wait_time:
            continue
        scf.cf.commander.send_setpoint(roll, pitch, yawrate, pwm_signal)

    scf.cf.commander.send_stop_setpoint()
    scf.cf.commander.send_notify_setpoint_stop()
    time.sleep(0.01)


if __name__ == '__main__':
    if PLOTTING:
        plot_thread = threading.Thread(target=plot_realtime, daemon=True)
        plot_thread.start()
    else:
        plot_thread = None

    restart([LOWERFLS_URI, UPPERFLS_URI])
    time.sleep(5)

    print("RESTART FINISHED")
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(URI_LIST, factory=factory) as swarm:
        swarm.reset_estimators()
        time.sleep(1)

        print("FLIGHT START")
        swarm.parallel_safe(start_logger)
        time.sleep(0.1)

        start_time = time.time()
        args_dict = {
            LOWERFLS_URI: [start_time, 0, DURATION, PWM_SIGNAL],
            UPPERFLS_URI: [start_time, UPPER_WAIT_TIME, DURATION, PWM_SIGNAL]
        }
        swarm.parallel_safe(async_flight, args_dict)
        stop_logger(LOGGERS)
        time.sleep(1)

    plot_metrics(config=CONFIG)

    try:
        stop_plot(plot_thread)
    except:
        pass
