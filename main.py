import datetime
import time

from DownwashCommander import DownwashCommander
from util.data_process import *
import config


if __name__ == '__main__':

    exp_config = config.Config

    file_prefix = datetime.datetime.fromtimestamp(time.time()).strftime("%Y_%m_%d_%H_%M")

    cfm = DownwashCommander(exp_config, logging_cfid=0, file_prefix=file_prefix, debug=exp_config.DEBUG)

    start_time = cfm.stationary_downwash()

    for i in range(exp_config.ITERATIONS):
        plot_metrics(f"metrics/{exp_config.CONFIG}/{file_prefix}_{i}.json")