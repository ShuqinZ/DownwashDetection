"""
Ramp motors and collect data using the system-id deck
"""
import argparse
import logging
import time
import yaml
from threading import Thread

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.localization import Localization


class CollectData:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        self._file = open("data.csv", "w+")
        self._file.write("timestamp,pwm,vbat[V]\n");

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='data', period_in_ms=10)
        self._lg_stab.add_variable('pwm.m1_pwm', 'uint16_t')
        self._lg_stab.add_variable('pm.vbatMV', 'uint16_t')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._ramp_motors).start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        # print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        #print('[%d][%s]: [%s][%s]' % (timestamp, logconf.name, data['pwm.m1_pwm'],data['pm.vbatMV']/ 1000))
        self._file = open("data.csv", "a")
        self._file.write("{},{},{}\n".format(timestamp,data['pwm.m1_pwm'],data['pm.vbatMV']/ 1000))

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def _ramp_motors(self):
        thrust_mult = 1
        thrust_step = 1000
        time_step = 1
        thrust = 0
        pitch = 0
        roll = 0
        yawrate = 0

        # # Unlock startup thrust protection
        # for i in range(0, 100):
        #     self._cf.commander.send_setpoint(0, 0, 0, 0)

        localization = Localization(self._cf)

        # self._cf.param.set_value('motor.batCompensation', 0)
        self._cf.param.set_value('motorPowerSet.m1', 0)
        self._cf.param.set_value('motorPowerSet.enable', 2)
        self._cf.param.set_value('system.forceArm', 1)

        while self.is_connected: #thrust >= 0:
            thrust += thrust_step * thrust_mult
            if thrust >= 65536 or thrust < 0:
            # if thrust >= 20000 or thrust < 0:
                thrust_mult *= -1
                thrust += thrust_step * thrust_mult
            print(thrust)
            # self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            localization.send_emergency_stop_watchdog()
            self._cf.param.set_value('motorPowerSet.m1', str(thrust))
            time.sleep(time_step)


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration", default="calibration.yaml", help="Input file containing the calibration")
    parser.add_argument("--uri", default="radio://0/80/2M/E7E7E7E7E7", help="URI of Crazyflie")
    args = parser.parse_args()

    # Only output errors from the logging framework
    logging.basicConfig(level=logging.ERROR)

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # collect data
    le = CollectData(args.uri)

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)