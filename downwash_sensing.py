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
import numpy as np
import argparse
from util.data_analysis import *

EXPERIMENT_NUM = 1
PREP_TIME = 5
UPPER_WAIT_TIME = 5
DURATION = 10
GAP_TIME = 5
THRUST_COMMAND = 10001  # Thrust from 10001-60000
LOGRATE = 100  # Hz

REALTIME_PLOT_DURATION = 2  # sec
REALTIME_PLOTTING = False

CONFIG = f"test_{THRUST_COMMAND}"

LOWERFLS_URI = 'radio://0/80/2M/E7E7E7E702'  # lower FLS
UPPERFLS_URI = 'radio://0/80/2M/E7E7E7E704'  # upper FLS

LOGGING_FLS = LOWERFLS_URI

URI_LIST = {
    LOWERFLS_URI,
    UPPERFLS_URI
}

deck_attached_event = Event()
logging.basicConfig(level=logging.ERROR)

log_vars = {}

log_vars_lock = Lock()

LOGGERS = []


def reset_log_vars():
    global log_vars

    with log_vars_lock:
        log_vars = {
            "Gyro_Accel": {
                "timestamp": [],
                "component": {
                    "Gyro": {
                        "range": [-45, 45],
                        "value": {
                            "stateEstimate.roll": {"type": "float", "unit": "deg", "data": []},
                            "stateEstimate.pitch": {"type": "float", "unit": "deg", "data": []},
                            "stateEstimate.yaw": {"type": "float", "unit": "deg", "data": []},
                        }
                    },
                    "Accel": {
                        "range": [-1, 1],
                        "value": {
                            "stateEstimate.ax": {"type": "float", "unit": "Gs", "data": []},
                            "stateEstimate.ay": {"type": "float", "unit": "Gs", "data": []},
                            "stateEstimate.az": {"type": "float", "unit": "Gs", "data": []}
                        }
                    }
                }
            },

            "Motor_Voltage": {
                "timestamp": [],
                "component": {
                    "pwm": {
                        "range": [0, 65535],
                        "value": {
                            "motor.m1": {"type": "uint16_t", "unit": "pwm_value", "data": []},
                            "motor.m2": {"type": "uint16_t", "unit": "pwm_value", "data": []},
                            "motor.m3": {"type": "uint16_t", "unit": "pwm_value", "data": []},
                            "motor.m4": {"type": "uint16_t", "unit": "pwm_value", "data": []},
                        },
                    },

                    "voltage": {
                        "range": [3000, 4200],
                        "value": {
                            "pm.vbatMV": {"type": "uint16_t", "unit": "mV", "data": []},
                        }
                    }
                }
            },
        }


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
    global log_vars

    with log_vars_lock:
        log_vars[logconf.name]["timestamp"].append(timestamp)
        for item in log_vars[logconf.name]["component"].keys():
            log_component = log_vars[logconf.name]["component"][item]
            for par in log_component["value"].keys():
                value = data[par]
                log_component["value"][par]["data"].append(value)


def save_log(config, file_prefix, start_time=0, duration=0, gap_time=0, iterations=1):
    global log_vars
    if not os.path.exists('metrics'):
        os.makedirs('metrics', exist_ok=True)

    if not os.path.exists('metrics/' + config):
        os.makedirs('metrics/' + config, exist_ok=True)

    end_time = start_time + duration * iterations + iterations * gap_time

    save_time = start_time + duration
    ite = 0

    while time.time() < end_time:
        if time.time() < save_time:
            continue
        filename = f"{file_prefix}_{ite:.0f}"
        save_time += gap_time + duration
        ite += 1

        with log_vars_lock:
            with open(f'metrics/{config}/{filename}.json', 'w') as f:
                json.dump(log_vars, f, indent=4)
        # print(f"SAVE LOG {filename}")

        reset_log_vars()


def plot_realtime(fps=50):
    """ Function to update real-time plots in a separate thread."""
    global log_vars, REALTIME_PLOTTING, REALTIME_PLOT_DURATION, LOGRATE

    n = int(np.floor(REALTIME_PLOT_DURATION * 1000 / LOGRATE))
    try:
        plt.ion()

        sub_plot_num = 0
        for name in log_vars.keys():
            sub_plot_num += len(name.split("_"))

        fig, axes = plt.subplots(nrows=sub_plot_num, ncols=1, figsize=(12, 10))

        while REALTIME_PLOTTING:
            with log_vars_lock:
                for ax in axes:
                    ax.clear()

                ax_index = 0
                for logs in log_vars.keys():
                    timeline = log_vars[logs]["timestamp"]
                    log_components = log_vars[logs]["component"]

                    if len(timeline) > 0:
                        time_axis = (np.array(timeline) - timeline[0]) / 1000
                        start_idx = max(0, len(time_axis) - n)
                        time_axis = time_axis[start_idx:]

                        for component in log_components.keys():
                            ax = axes[ax_index]
                            log_component = log_components[component]

                            for par in log_component["value"].keys():
                                data = np.array(log_component["value"][par]["data"])[start_idx:]
                                ax.plot(time_axis, data, label=f"{par} ({log_component['value'][par]['unit']})")
                            ax.set_xlabel('Time (s)')
                            ax.legend(loc="upper left")
                            ax.set_ylim(log_component["range"][0], log_component["range"][1])
                            ax.set_title(component)
                            ax_index += 1

            plt.tight_layout(h_pad=2)
            plt.draw()
            plt.pause(0.01)

        plt.close(fig)
    except:
        pass


def stop_plot(plot_thread):
    global REALTIME_PLOTTING
    REALTIME_PLOTTING = False
    if plot_thread and plot_thread.is_alive():
        plot_thread.join()


def start_logger(scf):
    global LOWERFLS_URI, LOGRATE
    if scf.cf.link_uri != LOGGING_FLS:
        return

    gyro_accel_logconf = LogConfig(name='Gyro_Accel', period_in_ms=1000 / LOGRATE)
    gyro_accel_logconf.add_variable('stateEstimate.roll', 'float')
    gyro_accel_logconf.add_variable('stateEstimate.pitch', 'float')
    gyro_accel_logconf.add_variable('stateEstimate.yaw', 'float')
    gyro_accel_logconf.add_variable('stateEstimate.ax', 'float')
    gyro_accel_logconf.add_variable('stateEstimate.ay', 'float')
    gyro_accel_logconf.add_variable('stateEstimate.az', 'float')
    scf.cf.log.add_config(gyro_accel_logconf)
    gyro_accel_logconf.data_received_cb.add_callback(
        lambda timestamp, data, logconf: log_callback(scf.cf.link_uri, timestamp, data, logconf)
    )

    # accel_logconf = LogConfig(name='Accel', period_in_ms=1000 / LOGRATE)
    # accel_logconf.add_variable('stateEstimate.ax', 'float')
    # accel_logconf.add_variable('stateEstimate.ay', 'float')
    # accel_logconf.add_variable('stateEstimate.az', 'float')
    # scf.cf.log.add_config(accel_logconf)
    # accel_logconf.data_received_cb.add_callback(
    #     lambda timestamp, data, logconf: log_callback(scf.cf.link_uri, timestamp, data, logconf)
    # )

    motor_logconf = LogConfig(name='Motor_Voltage', period_in_ms=1000 / LOGRATE)
    motor_logconf.add_variable('motor.m1', 'uint16_t')
    motor_logconf.add_variable('motor.m2', 'uint16_t')
    motor_logconf.add_variable('motor.m3', 'uint16_t')
    motor_logconf.add_variable('motor.m4', 'uint16_t')
    motor_logconf.add_variable('pm.vbatMV', 'uint16_t')
    scf.cf.log.add_config(motor_logconf)
    motor_logconf.data_received_cb.add_callback(
        lambda timestamp, data, logconf: log_callback(scf.cf.link_uri, timestamp, data, logconf)
    )

    # power_logconf = LogConfig(name='Power', period_in_ms=1000 / LOGRATE)
    # power_logconf.add_variable('pm.vbatMV', 'uint16_t')
    # scf.cf.log.add_config(power_logconf)
    # power_logconf.data_received_cb.add_callback(
    #     lambda timestamp, data, logconf: log_callback(scf.cf.link_uri, timestamp, data, logconf)
    # )

    # Start Logger
    gyro_accel_logconf.start()
    # accel_logconf.start()
    motor_logconf.start()
    # power_logconf.start()

    LOGGERS.extend([gyro_accel_logconf, motor_logconf])

    # LOGGERS.extend([gyro_logconf, accel_logconf, motor_logconf, power_logconf])


def stop_logger(loggers):
    for logger in loggers:
        logger.stop()


def async_flight(scf, start_time, iterations, stablize_time, wait_time, duration, gap_time, pwm_signal):
    scf.cf.commander.send_setpoint(0, 0, 0, 0)

    roll = 0.0
    pitch = 0.0
    yawrate = 0

    if stablize_time > 0:
        while time.time() < start_time + stablize_time:
            # print(f"STABALIZE {scf.cf.link_uri}")
            scf.cf.commander.send_setpoint(0.0, 0.0, 0, pwm_signal)
            pass

    start_time += stablize_time

    for i in range(iterations):
        ite_start_time = start_time + (duration + gap_time) * i
        ite_end_time = ite_start_time + duration

        wait_flag = False
        while time.time() < ite_end_time:
            if wait_time > 0 and time.time() < ite_start_time + wait_time:
                if wait_flag == False:
                    heart_beat_time = time.time() + 0.5
                    wait_flag = True
                    scf.cf.commander.send_setpoint(0, 0, 0, 0)
                
                elif wait_flag and time.time() > heart_beat_time:
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                        heart_beat_time += 0.5

                continue

            scf.cf.commander.send_setpoint(roll, pitch, yawrate, pwm_signal)

        print(f"Iteration {i} finished")

    scf.cf.commander.send_stop_setpoint()
    scf.cf.commander.send_notify_setpoint_stop()
    time.sleep(0.01)


if __name__ == '__main__':
    if REALTIME_PLOTTING:
        realtime_plot_thread = threading.Thread(target=plot_realtime, daemon=True)
        realtime_plot_thread.start()
    else:
        realtime_plot_thread = None

    reset_log_vars()
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

        # start_time, iterations, prep_time, wait_time, duration, gap_time, pwm_signal
        args_dict = {
            LOWERFLS_URI: [start_time, EXPERIMENT_NUM, PREP_TIME, 0, DURATION, GAP_TIME, THRUST_COMMAND],
            UPPERFLS_URI: [start_time + PREP_TIME, EXPERIMENT_NUM, 0, UPPER_WAIT_TIME, DURATION, GAP_TIME,
                           THRUST_COMMAND]
        }

        file_timestamp = datetime.datetime.fromtimestamp(start_time).strftime("%Y_%m_%d_%H_%M")

        save_log_thread = threading.Thread(target=save_log,
                                           kwargs={'config': CONFIG, 'file_prefix': file_timestamp,
                                                   'start_time': start_time + PREP_TIME, 'duration': DURATION,
                                                   'gap_time': GAP_TIME, 'iterations': EXPERIMENT_NUM}, daemon=True)
        save_log_thread.start()
        swarm.parallel_safe(async_flight, args_dict)
        stop_logger(LOGGERS)
        time.sleep(1)

    for i in range(EXPERIMENT_NUM):
        plot_metrics(f"metrics/{CONFIG}/{file_timestamp}_{i}.json")

    # plot_metrics(config=CONFIG)
    if save_log_thread and save_log_thread.is_alive():
        save_log_thread.join()

    try:
        stop_plot(realtime_plot_thread)
    except:
        pass
