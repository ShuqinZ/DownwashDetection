import json
import os
import numpy as np

import matplotlib.pyplot as plt

from pathlib import Path


def list_files(directory):
    try:
        files = [f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]
        return files
    except FileNotFoundError:
        print(f"Error: Directory '{directory}' not found.")
        return []
    except PermissionError:
        print(f"Error: Permission denied to access '{directory}'.")
        return []


def load_data(filepath):
    with open(filepath) as f:
        json_data = json.load(f)
    return json_data


def data_time_alignment(data, delay_time, log_gap=10):
    """
    Parameters:
    - delay_time: time diff comparing to reference timeline (ms)
    """

    data = np.array(data)

    offset_factor = delay_time / log_gap

    for i in range(1, len(data)):
        data[i] = data[i] + (data[i - 1] - data[i]) * offset_factor
    return data


def match_timeline(timeline_0, timeline_1, log_gap=10, time_offset=0):
    index_0, index_1 = int(np.ceil(time_offset / log_gap)), 0

    while index_0 < len(timeline_0) and index_1 < len(timeline_1):
        if timeline_0[index_0] <= timeline_1[index_1]:
            if abs(timeline_0[index_0] - timeline_1[index_1]) < log_gap:
                return index_0, index_1
            else:
                index_0 += 1
        else:
            index_1 += 1

    return -1, -1


def plot_metrics(filepath):
    log_vars = load_data(filepath)

    sub_plot_num = 0
    for name in log_vars.keys():
        sub_plot_num += len(name.split("_"))

    fig, axes = plt.subplots(nrows=sub_plot_num, ncols=1, figsize=(12, 10))

    ax_index = 0

    timeline = log_vars["timestamp"]
    log_components = log_vars["component"]

    if len(timeline) > 0:
        time_axis = (np.array(timeline) - timeline[0]) / 1000

        for component in log_components.keys():
            ax = axes[ax_index]
            log_component = log_components[component]

            for par in log_component["value"].keys():
                data = np.array(log_component["value"][par]["data"])
                ax.plot(time_axis, data, label=f"{par} ({log_component['value'][par]['unit']})")
            ax.set_xlabel('Time (s)')
            ax.legend(loc="upper left")
            ax.set_ylim(log_component["range"][0], log_component["range"][1])
            ax.set_title(component)
            ax_index += 1

    plt.tight_layout(h_pad=2)

    image_name = "".join(filepath.split('.')[:-1])
    plt.savefig(f'{image_name}.png', dpi=300)


def plot_general(filename, x_data, y_datas, y_names=None, x_label="Time(s)", y_label="", x_lim=None, y_lim=None):
    if y_names is None:
        y_names = ["" for _ in y_datas]

    fig, ax = plt.subplots(figsize=(8, 6))
    for y_data, y_name in zip(y_datas, y_names):
        ax.plot(x_data, y_data, label=y_name)

    ax.set_xlabel(x_label)
    # ax.legend(loc="upper left")
    ax.set_title(y_label)

    if x_lim is not None:
        ax.set_xlim(x_lim[0], x_lim[1])

    if y_lim is not None:
        ax.set_ylim(y_lim[0], y_lim[1])

    plt.tight_layout()
    plt.savefig(f'{filename}.png', dpi=300)
