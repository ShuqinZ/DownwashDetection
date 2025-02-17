import datetime

from DownwashCommander import DownwashCommander
from util.data_analysis import *
import config as exp_config


if __name__ == '__main__':

    cfm = DownwashCommander(exp_config, logging_cfid=0)

    start_time = cfm.stationary_downwash()

    file_prefix = datetime.datetime.fromtimestamp(start_time).strftime("%Y_%m_%d_%H_%M")

    for i in range(exp_config.ITERATIONS):
        plot_metrics(f"metrics/{exp_config.CONFIG}/{file_prefix}_{i}.json")