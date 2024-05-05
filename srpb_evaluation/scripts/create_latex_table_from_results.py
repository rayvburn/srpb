'''
Script that creates a .tex file with LaTeX command that can be used to put a table into the LaTeX document

Dependency:
sudo apt install python3-openpyxl
'''

from excel_sheet_utils import load_data_from_excel
from srpb_metrics import SrpbMetrics

import argparse
import json
import math

from pathlib import Path
from typing import List
from typing import Dict


# results: results of a specific scenarios aggregated into a single structure
# metrics: names (keys) of metrics to include in the LaTeX table
def create_latex_table(
    results: List[Dict[str, Dict[str, Dict[str, float]]]],
    metric_names: List[str],
    planner_names: List[str]
) -> str:
    # only keys known by the SrpbMetrics will be put into the LaTeX table; keys must match the ones used in the sheet
    srpb_metrics = SrpbMetrics()
    # select metrics from the predefined set stored in SrpbMetrics
    metrics_map = {}

    # by default, all metrics are included
    if not len(metric_names):
        metrics_map = srpb_metrics.get()
    else:
        metrics_map = srpb_metrics.get_metrics(metric_names)

    if not len(metrics_map):
        raise Exception(
            f"Aborting further execution as no metrics were selected to include in the LaTeX table. "
            f"See the script's usage instruction."
        )
    print(f"Selected `{len(metrics_map)}` metrics to include in the LaTeX table: `{metrics_map.keys()}`")

    # save the number of considered planners
    planners_num = len(planner_names)
    if not planners_num:
        # choose the set from the first scenario
        print(f"The input list of selected planners is empty! Selecting the planners appearing in the first scenario.")
        planner_names = results[0]['results'].keys()
        planners_num = len(planner_names)

    # evaluate whether valid data are available
    if not len(planner_names) or not planners_num:
        raise Exception(
            f"Aborting further execution since no planners were selected or appeared in the first scenario."
        )

    # retrieve number of evaluated scenarios
    scenarios_num = len(results)
    print(f"Selected '{planners_num}' planners with names `{planner_names}` for `{scenarios_num}` scenarios")


    def get_metric_value_for_planner(
        scenario_results: Dict[str, Dict[str, Dict[str, float]]],
        planner_name: str,
        metric_id: str
    ) -> float:
        """
        A nested function that returns a unified value once the required planner is nonexistent in the results.
        It is assumed that the nested dict given in scenario_results contains keys defining: planner name
        and dict nested further - a metric ID
        """
        metric_val = math.nan
        try:
            metric_val = scenario_results[planner_name][metric_id]
        except KeyError:
            print(
                f"Could not find a '{planner_name}' in a given results set. "
                f"Available planners are: '{scenario_results.keys()}'. "
                f"This might be intentional, returning NaN."
            )
        # value will store the value of the metric (if able to parse correctly)
        value = None
        if isinstance(metric_val, float):
            value = metric_val
        elif isinstance(metric_val, list):
            if not len(metric_val):
                raise Exception(f"'{metric_id}' metric value for the planner '{planner_name}' is an empty list")
            # arbitrarily selecting the first element in the list
            value = metric_val[0]
            # treat as a warning when the list is bigger than 1-element
            if len(metric_val) > 1:
                print(
                    f"\033[93m"
                    f"'{metric_id}' metric value for the planner '{planner_name}' is '{len(metric_val)}'-elem list, "
                    f"arbitrarily selecting the first element '{value}'"
                    "\033[0m"
                )
        else:
            raise Exception(f"'{metric_id}' metric value for the planner '{planner_name}' is of unsupported type")
        return value


    tex = str("")

    tex += ("% !TeX spellcheck = en_GB" + "\r\n")
    tex += ("% !TEX encoding = utf8" + "\r\n")
    tex += ("\r\n")

    # r forces to parse as raw string, ref: https://stackoverflow.com/a/46011113
    tex += (r"% Dependencies of the 'results' table" + "\r\n")
    tex += (r"\usepackage{graphicx} % \rotatebox" + "\r\n")
    tex += (r"\usepackage{multirow}" + "\r\n")
    tex += (r"\usepackage{diagbox}" + "\r\n")
    tex += ("\r\n")
    tex += (r"% Arguments:" + "\r\n")
    tex += (r"%  #1: (optional) table (default) or table*" + "\r\n")
    tex += (r"%  #2: size, e.g., 7.75cm is appropriate for 2-column article" + "\r\n")
    tex += (r"%  #3: table placement specifier, `ht` forces table at the end of the article document class" + "\r\n")
    tex += (r"\newcommand{\tabSrpbAutoResults}[3][table] {" + "\r\n")
    tex += ("\r\n")
    tex += (r"	% height of the header cells" + "\r\n")
    tex += (r"	\def \benchresultspheaderheight{1.60cm}" + "\r\n")
    tex += (r"" + "\r\n")
    tex += (r"	\begin{#1}[#3]" + "\r\n")
    tex += (r"		\centering" + "\r\n")
    tex += (r"		\resizebox{#2}{!}" + "\r\n")
    tex += (r"		{" + "\r\n")
    tex += (r"			% NOTE1: arg to parbox defines how high the text will start" + "\r\n")
    tex += (r"			% NOTE2: \cline{2-8} is a partial horizontal line, ref: https://tex.stackexchange.com/a/8555" + "\r\n")
    tex += (r"			\begin{tabular}" + "\r\n")

    # Prepare column for metric identifiers and units and for scenario identifiers
    tex += (r"			{||c||c||")
    # centered columns according to the number of planners
    for _ in range(planners_num):
        tex += (r"c|")
    # double vertical border and line end
    tex += (r"|}" + "\r\n")

    tex += (r"				\hline" + "\r\n")
    tex += (r"				% =============================== header" + "\r\n")
    tex += (r"				\multicolumn{2}{|c|}{ % spreads across metric and scenario ID" + "\r\n")
    tex += (r"					\diagbox" + "\r\n")
    tex += (r"						[width=3.15cm, height=2.00cm]" + "\r\n")
    tex += (r"						{\diagbox" + "\r\n")
    tex += (r"							[width=2.00cm, height=1.27cm]" + "\r\n")
    tex += (r"							{\raisebox{16pt}{\rotatebox{-33}{\hspace*{0.25cm}Metric}}}" + "\r\n")
    tex += (r"							{\raisebox{0pt}{\rotatebox{-33}{\hspace*{0.10cm}Scenario}}}" + "\r\n")
    tex += (r"						}" + "\r\n")
    tex += (r"						{\raisebox{-1.27cm}{\rotatebox{90}{Method}}}" + "\r\n")
    tex += (r"				}" + "\r\n")

    # header columns of planners - assuming that all results have the same planner entries
    for planner_name in planner_names:
        planner_name_latex_safe = planner_name.replace("_", "\_")
        tex += (r"				& \rotatebox[origin=c]{90}{" + "\r\n")
        tex += (r"					\parbox[c]{\benchresultspheaderheight}{" + "\r\n")
        tex += (r"						\centering" + "\r\n")
        # enter a name of the planner
        tex += (r"						\emph{" + str(planner_name_latex_safe) + r"}" + "\r\n")
        tex += (r"					}" + "\r\n")
        tex += (r"				}" + "\r\n")

    tex += (r"				% ===============================" + "\r\n")
    tex += (r"				\\ \hline\hline" + "\r\n")
    tex += (r"				% =============================== entries" + "\r\n")

    # iterate over metrics
    for metric_id in metrics_map.keys():
        tex += (r"" + "\r\n")
        tex += (r"				\multirow" + "\r\n")
        # total number of scenarios
        tex += (r"					{" + str(len(results)) + r"} % number of scenarios" + "\r\n")
        tex += (r"					{*}" + "\r\n")
        tex += (r"				{" + "\r\n")
        tex += (r"					\shortstack{" + "\r\n")
        # ID of the metric and its unit
        # whether to put the unit in a new line or not (when there are too few rows)
        if scenarios_num > 2:
            metric_name_and_unit = f"{metrics_map[metric_id]['tex_name']} \\ {metrics_map[metric_id]['tex_unit']}"
        else:
            metric_name_and_unit = f"{metrics_map[metric_id]['tex_name']} {metrics_map[metric_id]['tex_unit']}"
        tex += (r"						" + str(metric_name_and_unit) + "\r\n")
        tex += (r"					}" + "\r\n")
        tex += (r"				}" + "\r\n")
        tex += (r"				% =========" + "\r\n")

        # iterate over values of a specific metric for each scenario
        for scenario_num in range(scenarios_num):
            tex += (r"				% scenario " + str(scenario_num) + "\r\n")
            # add partial horizontal line
            if scenario_num > 0:
                # number reflects: metric col + scenario ID col + number of planner cols;
                # line starts from the 2nd column
                tex += (r"				\\ \cline{2-" + str(1 + 1 + planners_num) + "} % partial horizontal line" + "\r\n")
            # scenario identifier - ID taken from the inputs
            scenario_identifier = results[scenario_num]['name']
            # scenario_num
            tex += (r"				& \emph{" + str(scenario_identifier) + r"} % Scenario ID" + "\r\n")

            # find the best value among checked planners
            metric_values_among_planners = []
            for planner_name in planner_names:
                metric_val = get_metric_value_for_planner(results[scenario_num]['results'], planner_name, metric_id)
                metric_values_among_planners.append(metric_val)

            # select the best metric - the one with the smallest or largest value (excluding NaNs)
            if srpb_metrics.is_minimum_best(metric_id):
                metric_best_val = min(metric_values_among_planners)
            else:
                metric_best_val = max(metric_values_among_planners)

            # if all metric values are equal to the best - let's mark the best as 'invalid'
            if all(x == metric_best_val for x in metric_values_among_planners):
                metric_best_val = None

            # iterate over values of a specific metric for each planner
            for planner_name in planner_names:
                metric_val = get_metric_value_for_planner(results[scenario_num]['results'], planner_name, metric_id)
                # check for correctness/availability
                if not math.isnan(metric_val):
                    # format to 2 decimal points
                    metric_val_str = "{:.2f}".format(metric_val)
                else:
                    # indicate lack of valid data
                    metric_val_str = srpb_metrics.get_latex_missing_metric_value()

                # mark the best value unless it is equal to 0
                if metric_val == metric_best_val and metric_best_val != None:
                    # bold
                    tex += (r"				& " + r"\textbf{" + metric_val_str + r"}" + r" % " + str(planner_name) + "\r\n")
                else:
                    # normal font
                    tex += (r"				& " + metric_val_str + r" % " + str(planner_name) + "\r\n")

            # scenario separator
            tex += (r"				% =========" + "\r\n")

        # metric separator
        tex += (r"				\\ \hline" + "\r\n")
        tex += (r"				% ===============================" + "\r\n")

    # end of table
    tex += (r"			\end{tabular}" + "\r\n")
    tex += (r"		}" + "\r\n")
    tex += (r"		\caption{Automatically generated SRPB benchmark results}" + "\r\n")
    tex += (r"		\label{tab:bench:autoresults}" + "\r\n")
    tex += (r"	\end{#1}" + "\r\n")
    tex += (r"}" + "\r\n")

    return tex


###########################
#         main            #
###########################
if __name__ == "__main__":
    # Ref: https://stackoverflow.com/a/32763023
    cli = argparse.ArgumentParser()
    # positional arguments
    cli.add_argument("input", help="JSON string with scenario identifiers and paths to SRPB results sheets")
    cli.add_argument("output", help="path to the generated .tex file")
    # optional arguments
    cli.add_argument(
        "--metrics",
        nargs="*",
        type=str,
        default=[],
        help="space-separated list of metrics to include in the LaTeX table (all available are included by default)"
    )
    cli.add_argument(
        "--planners",
        nargs="*",
        type=str,
        default=[],
        help="space-separated list of planners to include in the LaTeX table (all appearing in the first results file \
            are included by default)"
    )
    # Usage example
    #   python3 create_latex_table_from_results.py \
    #     "{{"static": {{"sim": "path_static_sim", "real": "path_static_real"}}, "dynamic": {{"sim": "path_dynamic_sim", "real": "path_dynamic_real"}}}}" \
    #     ~/table.tex \
    #     --metrics m_obs m_chc \
    #     --planners dwa teb

    # parse the command line
    args = cli.parse_args()

    # location of the output file
    output_path = args.output

    # metrics to include in the table
    metric_names = args.metrics

    # selected planner set
    planner_names = args.planners

    # list of dicts {<name>, <path to excel>}
    inputs = []

    cmd_json = json.loads(args.input)
    for key in cmd_json.keys():
        inputs.append({'name': str(key), 'path': Path(cmd_json[key])})
    print("\tScript inputs:")
    print(*inputs, sep='\n')

    # list for storing results for all scenarios from inputs
    results_total = []

    # loop for processing Excel sheets for all input files
    for input_file in inputs:
        scenario_results = load_data_from_excel(input_file['path'])
        results_total.append({'name': input_file['name'], 'results': scenario_results})

    # generates a string representing LaTeX command that can be directly included and used in a LaTeX document
    results_table = create_latex_table(results_total, metric_names, planner_names)

    # save results to a file
    f = open(output_path, "w")
    f.write(results_table)
    f.close()

    # print info
    print(f"")
    print(f"LaTeX table saved to {output_path}")

    # print some usage notes
    print(f"")
    print(f"Include the file above and use the table in your LaTeX document using the provided command, e.g.,")
    print(f"\t" + r"\tabSrpbAutoResults[table*]{8cm}{}")
