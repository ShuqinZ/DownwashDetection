import time

from util import logger


def stationary_Flight(scf, start_time, iterations, prep_time, gap, wait_time, duration, command, debug=0, heart_beat=0.2):
    # The default heartbeat is 0.2, so if there is a package lost, the cf still have time to receive the new one.
    # The pitch needs to be set to the negative of the desired value
    # see line 95 of crazyflie-lib-python/cflib/crazyflie/commander.py

    scf.cf.commander.send_setpoint(0, 0, 0, 0)

    if isinstance(command, list):
        roll, pitch, yawrate, thrust = command
    else:
        roll = 0.0
        pitch = 0.0
        yawrate = 0
        thrust = command

    pitch = -pitch
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
                    heart_beat_time += heart_beat
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
                    heart_beat_time += heart_beat

        logger.debug(
            f"ID: {scf.cf.link_uri[-2:]}, Iteration {i} finished, Time: {time.time() - start_time:.2f}")

    scf.cf.commander.send_stop_setpoint()
    scf.cf.commander.send_notify_setpoint_stop()
    time.sleep(0.1)


def swirl_Flight(scf, start_time, iterations, prep_time, gap, wait_time, duration, thrust, debug=0, heart_beat=0.2):
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
                    heart_beat_time += heart_beat
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
                    heart_beat_time += heart_beat

        logger.debug(
            f"ID: {scf.cf.link_uri[-2:]}, Iteration {i} finished, Time: {time.time() - start_time:.2f}")

    scf.cf.commander.send_stop_setpoint()
    scf.cf.commander.send_notify_setpoint_stop()
    time.sleep(0.1)


def stable_Flight(scf, start_time, iterations, prep_time, gap, wait_time, duration, command, debug=0, heart_beat=0.2):
    # The default heartbeat is 0.2, so if there is a package lost, the cf still have time to receive the new one.
    # The pitch needs to be set to the negative of the desired value
    # see line 95 of crazyflie-lib-python/cflib/crazyflie/commander.py

    scf.cf.commander.send_setpoint(0, 0, 0, 0)

    if isinstance(command, list):
        roll, pitch, yawrate, thrust = command
    else:
        roll = 0.0
        pitch = 0.0
        yawrate = 0
        thrust = command

    pitch = -pitch
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
                    heart_beat_time += heart_beat
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
                    heart_beat_time += heart_beat

        logger.debug(
            f"ID: {scf.cf.link_uri[-2:]}, Iteration {i} finished, Time: {time.time() - start_time:.2f}")

    scf.cf.commander.send_stop_setpoint()
    scf.cf.commander.send_notify_setpoint_stop()
    time.sleep(0.1)