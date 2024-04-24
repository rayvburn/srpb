'''
Script that creates a .pdf file with a bar plot presenting results stored in a spreadsheet

Dependencies:
sudo apt install python3-openpyxl
sudo apt install python3-matplotlib
sudo apt install python3-numpy
'''

import argparse
import excel_sheet_utils
import matplotlib.pyplot as plt
import numpy as np
import yaml

from typing import Dict
from typing import List
from typing import Union


def filter_inner_keys(outer_dict: Dict, selected_keys: List[str]) -> Dict:
    """
    ChatGPT:
    This function takes the outer dictionary (outer_dict) and a list of selected keys (selected_keys).
    It then uses a nested dictionary comprehension to iterate over the outer dictionary and,
    for each inner dictionary, filter only the selected keys. The result is a new dictionary with the same outer
    structure but only containing the selected keys in the inner dictionaries.
    """
    return {outer_key: {inner_key: inner_dict[inner_key] for inner_key in selected_keys if inner_key in inner_dict}
            for outer_key, inner_dict in outer_dict.items()}


def prepare_data_for_plotting(data: Dict) -> Union[List[str], Dict[str, float]]:
    """
    Input data are constructed as follows:
    * dict with planner names as keys
      * dict with metric names as keys
        * metric values
    Output data are constructed as follows:
    * Union[
        list of metric names,
        dict of planner names with values of corresponding metrics (ordered according to the list)
    ]
    """
    if not data:
        raise Exception(f"Empty data provided!")

    planner_names = list(data.keys())
    if not planner_names:
        raise Exception(f"Inner dict is empty!")

    metric_names = list(data[planner_names[0]].keys())
    # NOTE: [0] because individual filtered values are stored in 1-element lists
    metrics_values = {
        planner: [
            data[planner].get(metric, None)[0]
            for metric in metric_names
        ] for planner in planner_names
    }
    return metric_names, metrics_values


def create_bar_plot(dataset: Dict, plot_cfg: Dict) -> plt.Figure:
    """
    Prepares a bar plot according to this tutorial:
    https://matplotlib.org/stable/gallery/lines_bars_and_markers/barchart.html#sphx-glr-gallery-lines-bars-and-markers-barchart-py
    """
    metric_name = dataset['metric']
    metric_planner_data = dataset['data']
    planner_names_list = list(metric_planner_data.keys())

    # find the number of scenarios as a size of a list with metric values of an arbitrary planner
    if not len(planner_names_list):
        raise Exception(f"Cannot proceed, data is empty")
    scenarios_num = len(metric_planner_data[planner_names_list[0]])

    # to find maximum (1-element metric sequences are expected)
    all_values = [value[0] for value in metric_planner_data.values()]
    # find the minimum and maximum values
    min_value = min(all_values)
    max_value = max(all_values)
    ylim_min = plot_cfg['figure']['ylim_min']
    ylim_max = plot_cfg['figure']['ylim_max_value_multiplier'] * max_value

    # abstract from inputs (this is an early version)
    planner_ids = metric_planner_data.keys()
    metric_values = metric_planner_data

    # the label locations
    x = np.arange(scenarios_num)

    # the width of the space reserved for bars
    width_scale = 1.0
    width = width_scale / len(metric_values) / 2
    # could be useful for the multi-scenario case
    multiplier = 0

    # prepare configuration entries
    font_cfg_legend = {
        'family': plot_cfg['legend']['fontfamily'],
        'size': plot_cfg['legend']['fontsize'],
    }
    # validate the color
    if not len(plot_cfg['figure']['bar_color']):
        plot_cfg['figure']['bar_color'] = None

    # Based on the example: https://matplotlib.org/stable/gallery/statistics/boxplot_vs_violin.html
    if plot_cfg['figure']['size_width'] == None or plot_cfg['figure']['size_height'] == None:
        fig, ax = plt.subplots()
    else:
        fig, ax = plt.subplots(figsize=(plot_cfg['figure']['size_width'], plot_cfg['figure']['size_height']))

    for attribute, measurement in metric_values.items():
        offset = width * multiplier
        rects = ax.bar(
            x + offset,
            measurement, width * plot_cfg['figure']['bar_width'],
            label=attribute,
            color=plot_cfg['figure']['bar_color'],
            edgecolor=plot_cfg['figure']['bar_edgecolor'],
            linewidth=plot_cfg['figure']['bar_edgewidth']
        )

        # Add labels manually instead of using "ax.bar_label()" function
        if plot_cfg['figure']['bar_height_labels']:
            for rect, label in zip(rects, measurement):
                height = rect.get_height()
                ax.text(
                    rect.get_x() + rect.get_width() / 2, # center the text
                    height,
                    str(plot_cfg['figure']['bar_height_label_format']).format(height), # limit decimal places
                    ha='center',
                    va='bottom',
                    fontdict=font_cfg_legend,
                    color='black'
                )
        # for calculating offsets
        multiplier += 1

    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_xlabel(plot_cfg['figure']['xlabel'], fontdict=font_cfg_legend)
    ax.set_ylabel(plot_cfg['figure']['ylabel'], fontdict=font_cfg_legend)
    ax.set_title(plot_cfg['figure']['title'], fontdict=font_cfg_legend)

    # Ref: https://stackoverflow.com/a/47893553
    idx = np.asarray([i * width for i in range(len(planner_ids))])
    ax.set_xticks(idx)
    ax.set_xticklabels(list(planner_ids), fontsize=font_cfg_legend['size'])
    ax.tick_params(axis='y', labelsize=font_cfg_legend['size'])

    # x labels rotation
    ax.tick_params(axis='x', rotation=plot_cfg['figure']['xlabels_rotation'])

    if plot_cfg['legend']['enable']:
        ax.legend(
            loc=plot_cfg['legend']['loc'],
            ncol=plot_cfg['legend']['ncol'],
            fontsize=font_cfg_legend['size']
        )
    ax.set_ylim(ylim_min, ylim_max)
    return fig


def create_remapped_dataset(data_orig: dict, names_orig_select: dict, remappings: dict):
    # remap the original IDs (names)
    data_remapped_keys = {}
    for name_orig in names_orig_select:
        label = remappings.get(name_orig)
        # check if found; also, if the remapped value already exists, do not overwrite
        if not label == None and not label in data_orig.keys():
            data_remapped_keys[label] = data_orig[name_orig]
        elif label == None:
            # if key was not found amid the remappings - do not change the original name
            data_remapped_keys[name_orig] = data_orig[name_orig]
    return data_remapped_keys


if __name__ == "__main__":
    # API is similar to the `create_box_plot_from_results` script
    # Ref: https://stackoverflow.com/a/32763023
    cli = argparse.ArgumentParser()
    # positional argument
    cli.add_argument("output", type=str, help="Path to the .pdf file with the plot to save")
    cli.add_argument(
        "--input",
        nargs="*",
        type=str,
        help="Path(s) to the SRPB results sheet(s). Their contents should be orthogonal - different planners in each"
    )
    cli.add_argument("--config", type=str, help="Path to the plot configuration file")
    cli.add_argument("--metric", type=str, help="ID of the metric")
    # optional arguments
    cli.add_argument(
        "--planners",
        nargs="*",
        type=str,
        help="Space-separated list of planners to include in the plot (all appearing in the results file \
            are included by default)"
    )

    # parse the command line
    args = cli.parse_args()

    sheet_paths = args.input
    config_path = args.config
    metric = args.metric
    output_path = args.output
    planners_raw = args.planners

    # load dict from the file
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    # this list will always be of size 1 (this is just a hack to pass it to `filter_inner_keys`)
    metric_name = [str(metric)]

    # load data from the spreadsheet(s)
    data_loaded = {}
    for sheet_path in sheet_paths:
        data_single_sheet = excel_sheet_utils.load_data_from_excel(sheet_path)
        # NOTE: this is probably not an optimal solution; ref: https://stackoverflow.com/a/26853961
        data_loaded_so_far = data_loaded
        data_loaded = {**data_loaded_so_far, **data_single_sheet}

    data_selected = filter_inner_keys(data_loaded, metric_name)

    # use all included in the input sheet
    if planners_raw == None or not len(planners_raw):
        print(f"Using planner IDs from the input spreadsheet as none were selected explicitly")
        planners_raw = []
        keys = data_loaded.keys()
        for planner_id in keys:
            planners_raw.append(planner_id)

    metric_name, planner_values = prepare_data_for_plotting(data_selected)
    # remap the original IDs (names) of the planners, store in the same dict
    planner_values = create_remapped_dataset(planner_values, planners_raw, cfg['labels'])

    # prepare dataset abstract (to eventually extend to a multi-plot scenario)
    dataset = {
        'metric': metric_name,
        'data': planner_values
    }

    # Create a new dictionary with only the desired keys
    plot_cfg_dict = {key: cfg[key] for key in ['figure', 'legend'] if key in cfg}

    # Create bar plot and return a figure handle
    fig = create_bar_plot(dataset, plot_cfg_dict)
    # blocking call until window with the plot is closed
    plt.show()

    # save the figure
    fig.savefig(output_path, bbox_inches='tight', pad_inches=0)
    print(f"Plot saved to `{output_path}` file")
