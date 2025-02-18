import json
import os
import threading
import queue
import time
import datetime
from util import logger


class CFDataLogger:
    def __init__(self, config, duration, gap):

        self._config = config
        self._start_time = None
        self._duration = duration
        self._gap = gap
        self._file_prefix = ""

        self.log_vars = None
        self.reset_log_vars()

        self._init_log_directory()

        self.log_queue = queue.Queue()
        self.running = True

        self.log_thread = threading.Thread(target=self._process_logs, daemon=True)

    def start_logging(self, start_time):
        self._start_time = start_time
        self._file_prefix = datetime.datetime.fromtimestamp(start_time).strftime("%Y_%m_%d_%H_%M")
        self.log_thread.start()

    def _init_log_directory(self):

        if not os.path.exists('metrics'):
            os.makedirs('metrics', exist_ok=True)

        if not os.path.exists('metrics/' + self._config):
            os.makedirs('metrics/' + self._config, exist_ok=True)

    def _process_logs(self):
        """ Continuously process logs from the queue """

        iterations = 0
        log_start_time = self._start_time + self._gap
        save_time = self._start_time + self._gap + self._duration

        # logger.debug(f"LOG Start Time: {log_start_time-self._start_time}, LOG Save Time: {save_time - self._start_time}")

        while self.running or not self.log_queue.empty():
            try:
                log_item = self.log_queue.get(timeout=0.01)  # Waits for a log entry

                timestamp, name, data = log_item


                # logger.debug(f"Timestamp: {timestamp}, Current Time: {time.time() - self._start_time:.2f} Start Time: {log_start_time- self._start_time}, Save Time: {save_time - self._start_time}")

                if time.time() > log_start_time:

                    # if it's time to save, skip logging it and
                    if time.time() > save_time:
                        filename = self._file_prefix + f"_{iterations}"
                        self._save_log(filename)
                        # logger.info(f"LOG SAVED for iteration: {iterations}")

                        self.reset_log_vars()

                        logger.info(
                            f"Time: {time.time() - self._start_time:.2f}, ITERATION: {iterations}, LOG Start Time: {log_start_time - self._start_time}, LOG Save Time: {save_time - self._start_time}")
                        
                        iterations += 1
                        save_time += self._gap + self._duration
                        log_start_time += self._gap + self._duration

                        continue

                    self.log_vars["timestamp"].append(timestamp)
                    for item in self.log_vars["component"].keys():
                        log_component = self.log_vars["component"][item]
                        for par in log_component["value"].keys():
                            value = data[par]
                            log_component["value"][par]["data"].append(value)

            except queue.Empty:
                continue

    def _save_log(self, filename):

        with open(f'metrics/{self._config}/{filename}.json', 'w') as f:
            json.dump(self.log_vars, f, indent=4)

        self.reset_log_vars()


    def log(self, timestamp, name, data):
        # logger.debug("LOG IN QUEUE")
        self.log_queue.put((timestamp, name, data))

    def stop(self):
        """ Stops the logger thread gracefully """
        self.running = False
        self.log_thread.join()

    def reset_log_vars(self):
        self.log_vars = {
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
                "Motor": {
                        "range": [0, 65535],
                        "value": {
                            "motor.m1": {"type": "uint16_t", "unit": "pwm_value", "data": []},
                            "motor.m2": {"type": "uint16_t", "unit": "pwm_value", "data": []},
                            "motor.m3": {"type": "uint16_t", "unit": "pwm_value", "data": []},
                            "motor.m4": {"type": "uint16_t", "unit": "pwm_value", "data": []},
                        },
                    },

                "Battery": {
                    "range": [3000, 4200],
                    "value": {
                        "pm.vbatMV": {"type": "uint16_t", "unit": "mV", "data": []},
                    }
                }
            }
        }

    # def stop_plot(self, plot_thread):
    #     if plot_thread and plot_thread.is_alive():
    #         plot_thread.join()
    #
    # def plot_realtime(fps=50):
    #     """ Function to update real-time plots in a separate thread."""
    #     global log_vars, REALTIME_PLOTTING, REALTIME_PLOT_DURATION, LOGRATE
    #
    #     n = int(np.floor(REALTIME_PLOT_DURATION * 1000 / LOGRATE))
    #     try:
    #         plt.ion()
    #
    #         sub_plot_num = 0
    #         for name in log_vars.keys():
    #             sub_plot_num += len(name.split("_"))
    #
    #         fig, axes = plt.subplots(nrows=sub_plot_num, ncols=1, figsize=(12, 10))
    #
    #         while REALTIME_PLOTTING:
    #             with log_vars_lock:
    #                 for ax in axes:
    #                     ax.clear()
    #
    #                 ax_index = 0
    #                 for logs in log_vars.keys():
    #                     timeline = log_vars[logs]["timestamp"]
    #                     log_components = log_vars[logs]["component"]
    #
    #                     if len(timeline) > 0:
    #                         time_axis = (np.array(timeline) - timeline[0]) / 1000
    #                         start_idx = max(0, len(time_axis) - n)
    #                         time_axis = time_axis[start_idx:]
    #
    #                         for component in log_components.keys():
    #                             ax = axes[ax_index]
    #                             log_component = log_components[component]
    #
    #                             for par in log_component["value"].keys():
    #                                 data = np.array(log_component["value"][par]["data"])[start_idx:]
    #                                 ax.plot(time_axis, data, label=f"{par} ({log_component['value'][par]['unit']})")
    #                             ax.set_xlabel('Time (s)')
    #                             ax.legend(loc="upper left")
    #                             ax.set_ylim(log_component["range"][0], log_component["range"][1])
    #                             ax.set_title(component)
    #                             ax_index += 1
    #
    #             plt.tight_layout(h_pad=2)
    #             plt.draw()
    #             plt.pause(0.01)
    #
    #         plt.close(fig)
    #     except:
    #         pass