'''
Script that creates a .pdf file with a violin/box plot presenting results

Dependencies:
sudo apt install python3-openpyxl
sudo apt install python3-matplotlib
'''

import argparse
import excel_sheet_utils
import matplotlib.pyplot as plt
import yaml


def remap_names(orig_names: list, remappings: dict):
    # remap the original IDs (names)
    names = []
    for name in orig_names:
        label = remappings.get(name)
        # check if found; if not - do not change the name
        if not label == None:
            names.append(label)
        else:
            names.append(name)
    return names


if __name__ == "__main__":
    # Ref: https://stackoverflow.com/a/32763023
    cli = argparse.ArgumentParser()
    # positional arguments
    cli.add_argument("input", type=str, help="Path to the SRPB results sheet")
    cli.add_argument("output", type=str, help="Path to the .pdf file with the plot to save")
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
    # Example usage:
    # python3 \
    #   create_box_plot_from_results.py \
    #     <full path to the automatically generated .xlsx file> \
    #     test.pdf \
    #     --config create_box_plot_from_results.yaml \
    #     --metric m_mef \
    #     --planners teb dwa
    #

    # parse the command line
    args = cli.parse_args()

    sheet_path = args.input
    config_path = args.config
    metric = args.metric
    output_path = args.output
    planners_raw = args.planners

    # load dict from the file
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    # load data from the spreadsheet (either only filtered data - bar plot, or all data for statistical presentation)
    raw_data = excel_sheet_utils.load_raw_data_from_excel(sheet_path)

    # use all included in the input sheet
    if planners_raw == None or not len(planners_raw):
        print(f"Using planner IDs from the input spreadsheet as none were selected explicitly")
        planners_raw = []
        keys = raw_data.keys()
        for planner_id in keys:
            planners_raw.append(planner_id)

    #
    # Structure of data per individual metric:
    #
    # data = [
    #     # -> ..., <subsequent values of a certain metric>, ...,
    #     [0.0, 0.0, 0.0, 0.0, 0.0], # ^
    #     [0.0, 0.0, 0.0, 0.0, 0.0], # data of planner X
    #     [0.0, 0.0, 0.0, 0.0, 0.0], # data of planner Y
    #     [0.0, 0.0, 0.0, 0.0, 0.0], # data of planner Z
    #     [0.0, 0.0, 0.0, 0.0, 0.0], # data of planner W
    #     [0.0, 0.0, 0.0, 0.0, 0.0]  # |
    # ]
    #
    # prepare the loaded data according to the example above
    data = []
    for planner in planners_raw:
        data.append(raw_data[planner][metric])

    # remap the original IDs (names) of the planners
    planners = remap_names(planners_raw, cfg['labels'])

    # Based on the example: https://matplotlib.org/stable/gallery/statistics/boxplot_vs_violin.html
    if cfg['figure']['size_width'] == None or cfg['figure']['size_height'] == None:
        fig, ax = plt.subplots()
    else:
        fig, ax = plt.subplots(figsize=(cfg['figure']['size_height'], cfg['figure']['size_width']))

    if cfg['plot_type'] == "violin":
        ax.violinplot(
            data,
            showmeans=False,
            showmedians=True
        )
    elif cfg['plot_type'] == "box":
        ax.boxplot(data)
    else:
        raise Exception(f"Unknown plot_type")

    ax.set_title(cfg['figure']['title'])
    ax.xaxis.grid(cfg['figure']['xaxis_grid'])
    ax.yaxis.grid(cfg['figure']['yaxis_grid'])
    ax.set_xlabel(cfg['figure']['xlabel'])
    ax.set_ylabel(cfg['figure']['ylabel'])
    ax.set_xticks(range(1, len(data) + 1))
    ax.set_xticklabels(planners)
    plt.show()

    # save the figure
    fig.savefig(output_path, bbox_inches='tight', pad_inches=0)
    print(f"Plot saved to `{output_path}` file")
