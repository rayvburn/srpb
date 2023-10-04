'''
Script that creates a .tex file with LaTeX command that can be used to put a table into the LaTeX document

Dependency:
sudo apt install python3-openpyxl
'''

import excel_sheet_defines
import json
import sys

from openpyxl import load_workbook
from pathlib import Path

from typing import List
from typing import Dict


def load_data_from_excel(path: Path) -> Dict[str, Dict[str, float]]:
    # reading from sheets follows the convention introduced in `create_excel_from_results.calculate_sheet` script

    # NOTE: data_only allows to read values instead of formulas
    wb = load_workbook(filename = str(path), data_only=True)
    # access specific sheet
    sheet = wb[excel_sheet_defines.SHEET_NAME]
    # upper left cell - cell that initiates results table
    col_init = excel_sheet_defines.RESULT_INIT_COL
    row_init = excel_sheet_defines.RESULT_INIT_ROW

    col_metric_ids = col_init

    # lambda that creates col-row tuple to access cell in a sheet
    make_sheet_cell = lambda row, col : str(col + row)
    # lambda that works for chars in A-Z range
    increment_char = lambda c : chr(ord(c) + 1)
    # lambda that increments integer stored as string
    increment_row = lambda row : str(int(row) + 1)
    # lambda that retrieves value of sheet's cell
    get_sheet_val = lambda sheet, row, col : sheet[make_sheet_cell(col=col, row=row)].value
    # lambda that checks if certain cell in the sheet is empty
    sheet_cell_empty = lambda sheet, row, col : get_sheet_val(sheet=sheet, row=row, col=col) == None

    # local database with planners' metrics
    data = {}

    # to iterate over planners - skip first, avoiding 'Planner'
    col_iter = increment_char(col_init)
    # to iterate over metrics
    row_iter = increment_row(row_init)
    # iterate over planners (names)
    while not sheet_cell_empty(sheet=sheet, row=row_init, col=col_iter):
        # save planner name for later use in database
        planner_name = str(get_sheet_val(sheet=sheet, row=row_init, col=col_iter))
        # got a valid planner name - iterate over saved metrics
        metric_names = []
        metric_values = []
        while not sheet_cell_empty(sheet=sheet, row=row_iter, col=col_metric_ids):
            # obtain sheet values
            metric_name = get_sheet_val(sheet=sheet, row=row_iter, col=col_metric_ids)
            metric_value = get_sheet_val(sheet=sheet, row=row_iter, col=col_iter)
            # evaluate the correctness of read data
            if metric_name == None or metric_value == None:
                raise Exception(
                    f"The metric ID cell `{col_metric_ids}{row_iter}` contains `{metric_name}` "
                    f"and the metric value cell `{col_iter}{row_iter}` contains `{metric_value}`. "
                    f"Cannot proceed with such values. Check whether the cells in your spreadsheet are empty. "
                    f"If they aren't, try to open the sheet and simply save it using Excel or LibreOffice Calc."
                )
            # collect
            metric_names.append(str(metric_name))
            metric_values.append(float(metric_value))
            # try to proceed to the next metric
            row_iter = increment_row(row_iter)

        # finished collecting metrics for a given planner
        metrics = dict(zip(metric_names, metric_values))
        # append to overall results
        result = {planner_name: metrics}
        data.update(result)

        # try to proceed to the next planner
        col_iter = increment_char(col_iter)
        # reset metrics row 'pointer'
        row_iter = increment_row(row_init)

    return data

# results: results of a specific scenarios aggregated into a single structure
# metrics: names (keys) of metrics to include in the LaTeX table
def create_latex_table(results: List[Dict[str, Dict[str, Dict[str, float]]]], metric_names: List[str]) -> str:
    # only keys from this map will be put into the LaTeX table; keys must match the ones used in the Excel sheet
    METRIC_LATEX_MAP = {
        'm_obs':  r"$m_{\mathrm{obs}}$    \\ $\left[ \% \right]$",
        'm_mef':  r"$m_{\mathrm{mef}}$    \\ $\left[ \mathrm{s} \right]$",
        'm_path': r"$m_{\mathrm{plin}}$   \\ $\left[ \mathrm{m} \right]$",
        'm_chc':  r"$m_{\mathrm{chc}}$    \\ $\left[ \mathrm{rad} \right]$",
        'm_cef':  r"$m_{\mathrm{cef}}$    \\ $\left[ 10^{-3} \cdot \mathrm{s} \right]$",
        'm_cre':  r"$m_{\mathrm{cre}}$    \\ $\left[ 10^{-3} \cdot \mathrm{s} \right]$",
        'm_vsm':  r"$m_{\mathrm{vsm}}$    \\ $\left[ \mathrm{\frac{m}{s^2}} \right]$",
        'm_hsm':  r"$m_{\mathrm{hsm}}$    \\ $\left[ \mathrm{\frac{rad}{s^2}} \right]$",
        'm_osc':  r"$m_{\mathrm{osc}}$    \\ $\left[ \% \right]$",
        'm_bwd':  r"$m_{\mathrm{bwd}}$    \\ $\left[ \% \right]$",
        'm_inp':  r"$m_{\mathrm{iprot}}$  \\ $\left[ \% \right]$",
        'm_psi':  r"$m_{\mathrm{psi}}$    \\ $\left[ \% \right]$",
        'm_fsi':  r"$m_{\mathrm{fsi}}$    \\ $\left[ \% \right]$",
        'm_dir':  r"$m_{\mathrm{dir}}$    \\ $\left[ \% \right]$",
        'm_psd':  r"$m_{\mathrm{psd}}$    \\ $\left[ \% \right]$"
    }
    # select metrics from the predefined set, i.e., METRIC_LATEX_MAP
    metrics_map = {}
    # by default, all metrics are included
    if not len(metric_names):
        metrics_map = METRIC_LATEX_MAP
    else:
        for name in metric_names:
            if not name in METRIC_LATEX_MAP.keys():
                raise Exception(
                    f"Could not find {name} in the SRPB metrics map. Available metric names are: {METRIC_LATEX_MAP.keys()}"
                )
            metrics_map[name] = METRIC_LATEX_MAP[name]

    if not len(metrics_map):
        raise Exception(
            f"Aborting further execution as no metrics were selected to include in the LaTeX table. "
            f"See the script's usage instruction."
        )
    print(f"Selected `{len(metrics_map)}` metrics to include in the LaTeX table: `{metrics_map.keys()}`")

    # retrieve names of planners assuming that all scenarios results have the same planner entries;
    # choose from the first scenario
    planner_names = results[0]['results'].keys()
    # save number of checked planners
    planners_num = len(planner_names)
    # retrieve number of evaluated scenarios
    scenarios_num = len(results)

    # evaluate whether valid data are available
    if not len(planner_names) or not planners_num:
        raise Exception(
            f"Aborting further execution as no planners are found in the results. "
        )

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
    tex += (r"			{||c||c||c|c|c|c|c|c||}" + "\r\n")
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
        tex += (r"				& \rotatebox[origin=c]{90}{" + "\r\n")
        tex += (r"					\parbox[c]{\benchresultspheaderheight}{" + "\r\n")
        tex += (r"						\centering" + "\r\n")
        # enter a name of the planner
        tex += (r"						\emph{" + str(planner_name) + r"}" + "\r\n")
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
        tex += (r"						" + str(metrics_map[metric_id]) + "\r\n")
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
                metric_val = results[scenario_num]['results'][planner_name][metric_id]
                metric_values_among_planners.append(metric_val)
            # NOTE: by default, the best metric is the one with the smallest value
            metric_best_val = min(metric_values_among_planners)

            # if all metric values are equal to the best - let's mark the best as 'invalid'
            if all(x == metric_best_val for x in metric_values_among_planners):
                metric_best_val = None

            # iterate over values of a specific metric for each planner
            for planner_name in planner_names:
                metric_val = results[scenario_num]['results'][planner_name][metric_id]
                # format to 2 decimal points
                metric_val_str = "{:.2f}".format(metric_val)
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
    # command line arguments
    if len(sys.argv) == 1:
        print(f'Usage: ')
        print(
              f'  python3 {sys.argv[0]}  '
              f'<JSON string with scenario identifiers and paths to SRPB results sheets>  '
              f'<path to the generated .tex file>  <space-separated list of metrics to include in the LaTeX table '
              f'(all available are included by default)>'
            )
        print(f'')
        print(f'Example:')
        print(f'  python3 {sys.argv[0]} "{{"static": {{"sim": "path_static_sim", "real": "path_static_real"}}, "dynamic": {{"sim": "path_dynamic_sim", "real": "path_dynamic_real"}}}}" ~/table.tex m_obs m_chc')
        exit(0)

    # location of the output file
    output_path = sys.argv[2]

    # metrics to include in the table
    metric_names = sys.argv[3:]

    # list of dicts {<name>, <path to excel>}
    inputs = []

    cmd_json = json.loads(sys.argv[1])
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
    results_table = create_latex_table(results_total, metric_names)

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
