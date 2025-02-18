import datetime
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.power_switch import PowerSwitch
from util.CFDataLogger import CFDataLogger
from util import logger


def stationary_Flight(scf, start_time, iterations, prep_time, gap, wait_time, duration, thrust, debug=0):
    scf.cf.commander.send_setpoint(0, 0, 0, 0)

    roll = 0.0
    pitch = 0.0
    yawrate = 0

    command_change = True

    iteration_start_time = start_time + prep_time

    if prep_time > 0 and wait_time == 0:
        while time.time() < iteration_start_time:
            if debug > 0:
                if debug == 1:
                    logger.debug(
                        f"ID: {scf.cf.link_uri[-2:]}, Send Command: Stabilize, Time: {time.time() - start_time:.2f}")
                elif debug == 2 and command_change:
                    command_change = False
                    logger.debug(
                        f"ID: {scf.cf.link_uri[-2:]}, Send Command: Stabilize, Time: {time.time() - start_time:.2f}")
                # elif debug == 3:
                continue
            else:
                scf.cf.commander.send_setpoint(0.0, 0.0, 0, thrust)

    for i in range(iterations):
        ite_start_time = iteration_start_time + (duration + gap) * i
        ite_end_time = ite_start_time + duration

        moving = True
        command_change = True

        heart_beat_time = time.time()

        while time.time() < ite_end_time:

            # if cf should wait and is waiting
            if wait_time > 0 and time.time() < ite_start_time + wait_time:
                if moving:
                    heart_beat_time = time.time()
                    moving = False
                    command_change = True

                if time.time() >= heart_beat_time:
                    if debug > 0:
                        if debug == 1:
                            logger.debug(
                                f"ID: {scf.cf.link_uri[-2:]}, Send Command: Wake, Time: {time.time() - start_time:.2f}")
                        elif debug == 2 and command_change:
                            command_change = False
                            logger.debug(
                                f"ID: {scf.cf.link_uri[-2:]}, Send Command: Wake, Time: {time.time() - start_time:.2f}")
                        # elif debug == 3:
                        continue
                    else:
                        scf.cf.commander.send_setpoint(0, 0, 0, 0)
                    heart_beat_time += 0.5
                continue

            # if cf should move and is moving
            else:
                if not moving:
                    heart_beat_time = time.time()
                    moving = True
                    command_change = True

                if time.time() >= heart_beat_time:
                    if debug > 0:
                        if debug == 1:
                            logger.debug(
                                f"ID: {scf.cf.link_uri[-2:]}, Send Command: Move, Time: {time.time() - start_time:.2f}")
                        elif debug == 2 and command_change:
                            command_change = False
                            logger.debug(
                                f"ID: {scf.cf.link_uri[-2:]}, Send Command: Move, Time: {time.time() - start_time:.2f}")
                        # elif debug == 3:
                        continue
                    else:
                        scf.cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
                    heart_beat_time += 0.5

        logger.debug(
                    f"ID: {scf.cf.link_uri[-2:]}, Iteration {i} finished, Time: {time.time() - start_time:.2f}")

    scf.cf.commander.send_stop_setpoint()
    scf.cf.commander.send_notify_setpoint_stop()
    time.sleep(0.1)


class DownwashCommander:
    def __init__(self, exp_config, logging_cfid=0, log_rate=100, file_prefix="", debug=[0, 0]):
        self._exp_config = exp_config
        self._cf_loggers = []
        self._logging_cfid = logging_cfid
        self._log_rate = log_rate  # Hz
        self._start_time = None
        self.logger = CFDataLogger(exp_config.CONFIG, exp_config.DURATION, exp_config.GAP, file_prefix)
        self._debug = debug

    def restart(self, cfid=None):
        if cfid is None:
            uri = self._exp_config.URI_LIST
        else:
            uri = self._exp_config.URI_LIST[cfid]

        if isinstance(uri, list):
            for link in uri:
                PowerSwitch(link).stm_power_cycle()
        else:
            PowerSwitch(uri).stm_power_cycle()

    def log_callback(self, uri, timestamp, data, logconf):
        if uri == self._exp_config.URI_LIST[self._logging_cfid]:
            if self._start_time is None:
                start_timestamp = timestamp/1000
                self._start_time = time.time()
                self.logger.start_logging(start_timestamp)

            self.logger.log(timestamp, logconf.name, data)

    def init_cflogger(self, scf):
        if scf.cf.link_uri != self._exp_config.URI_LIST[self._logging_cfid]:
            return

        logger = LogConfig(name='Gyro_Motor_Battery', period_in_ms=1000 / self._log_rate)
        logger.add_variable('stateEstimate.roll', 'float')
        logger.add_variable('stateEstimate.pitch', 'float')
        logger.add_variable('stateEstimate.yaw', 'float')
        logger.add_variable('motor.m1', 'uint16_t')
        logger.add_variable('motor.m2', 'uint16_t')
        logger.add_variable('motor.m3', 'uint16_t')
        logger.add_variable('motor.m4', 'uint16_t')
        logger.add_variable('pm.vbatMV', 'uint16_t')

        scf.cf.log.add_config(logger)
        logger.data_received_cb.add_callback(
            lambda timestamp, data, logconf: self.log_callback(scf.cf.link_uri, timestamp, data, logconf)
        )

        self._cf_loggers.extend([logger])

    def start_logger(self):
        for logger in self._cf_loggers:
            logger.start()

    def stop_logger(self):
        for logger in self._cf_loggers:
            logger.stop()

        self.logger.stop()

    def stationary_downwash(self):
        cflib.crtp.init_drivers()

        self.restart()
        time.sleep(5)
        factory = CachedCfFactory(rw_cache='./cache')
        with Swarm(self._exp_config.URI_LIST, factory=factory) as swarm:
            logger.info("RESTART FINISHED")

            swarm.reset_estimators()
            time.sleep(1)

            logger.info("FLIGHT START")
            swarm.parallel_safe(self.init_cflogger)
            time.sleep(0.1)

            self.start_logger()

            while self._start_time is None:
                continue

            logger.info("Settle Start Time")

            # start_time, iterations, prep_time, gap, wait_time, duration, thrust, debug=0
            args_dict = {
                self._exp_config.URI_LIST[0]: [self._start_time, self._exp_config.ITERATIONS, self._exp_config.GAP,
                                               self._exp_config.GAP, 0,
                                               self._exp_config.DURATION, self._exp_config.THRUST, self._debug[0]],
                self._exp_config.URI_LIST[1]: [self._start_time, self._exp_config.ITERATIONS, self._exp_config.GAP,
                                               self._exp_config.GAP, self._exp_config.WAIT_TIME, 
                                               self._exp_config.DURATION, self._exp_config.THRUST, self._debug[1]]
            }

            swarm.parallel_safe(stationary_Flight, args_dict)
            time.sleep(2)
            self.stop_logger()
