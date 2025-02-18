import datetime

from DownwashCommander import DownwashCommander
from util.data_process import *
import config


if __name__ == '__main__':

    exp_config = config.Config

    cfm = DownwashCommander(exp_config, logging_cfid=0, debug=exp_config.DEBUG)

    start_time = cfm.stationary_downwash()

    file_prefix = datetime.datetime.fromtimestamp(start_time).strftime("%Y_%m_%d_%H_%M")

    for i in range(exp_config.ITERATIONS):
        plot_metrics(f"metrics/{exp_config.CONFIG}/{file_prefix}_{i}.json")