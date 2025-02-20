import json
import os
import numpy as np
import ruptures as rpt

import matplotlib.pyplot as plt

from pathlib import Path


def bcpd(data, num):
    # Apply Bayesian Change Point Detection
    algo = rpt.Window(model="l2", width=10).fit(np.array(data))  # Binary Segmentation Algorithm
    change_points = algo.predict(n_bkps=(num+2))  # Detect one change point

    results = []
    for c in change_points[:-1]:
        if c == 0 or c == change_points[-1]:
            continue
        results.append(c)

        if len(results) >= num:
            break

    return results


def is_2d_list(list2d):
    return all(isinstance(row, list) or isinstance(row, np.ndarray) for row in list2d)


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


def plot_metrics(filepath, ):
    log_vars = load_data(filepath)

    timeline = log_vars["timestamp"]
    log_components = log_vars["component"]

    fig, axes = plt.subplots(nrows=len(log_components.keys()), ncols=1, figsize=(12, 10))

    ax_index = 0

    if len(timeline) > 0:
        time_axis = (np.array(timeline) - timeline[0]) / 1000

        for component in log_components.keys():
            ax = axes[ax_index]
            log_component = log_components[component]

            for par in log_component["value"].keys():
                data = np.array(log_component["value"][par]["data"])
                ax.plot(time_axis, data, label=f"{par} ({log_component['value'][par]['unit']})")

                if par == "Battery":
                    ax.axhline(y=3400, color='r', alpha=0.5, linestyle='--', linewidth=2, label='Voltage Baseline')
            ax.set_xlabel('Time (s)')
            ax.legend(loc="upper left")
            ax.set_ylim(log_component["range"][0], log_component["range"][1])
            ax.set_title(component)
            ax_index += 1

    plt.tight_layout(h_pad=2)

    image_name = "".join(filepath.split('.')[:-1])
    plt.savefig(f'{image_name}.png', dpi=300)


def plot_general(filename, x_data, y_datas, line_names=None, x_label="Time(s)", y_label="", x_lim=None, y_lim=None,
                 plot=None):
    if line_names is None:
        line_names = ["" for _ in y_datas]

    if plot is None:
        fig, ax = plt.subplots(figsize=(8, 6))
    else:
        fig, ax = plot

    if is_2d_list(y_datas):
        for y_data, y_name in zip(y_datas, line_names):
            ax.plot(x_data, y_data, label=y_name)
    else:
        ax.plot(x_data, y_datas, label=line_names)

    ax.set_xlabel(x_label)
    ax.legend(loc="upper left")
    ax.set_title(y_label)

    if x_lim is not None:
        ax.set_xlim(x_lim[0], x_lim[1])

    if y_lim is not None:
        ax.set_ylim(y_lim[0], y_lim[1])

    if filename and filename != "":
        plt.tight_layout()
        plt.savefig(f'{filename}.png', dpi=300)
