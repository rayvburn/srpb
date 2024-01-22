from openpyxl import load_workbook
from pathlib import Path
from typing import Dict

# name of the sheet in the Excel file with the results
SHEET_NAME = "SRPB"

# upper left cell - cell that initiates results table
RESULT_INIT_COL = 'A'
RESULT_INIT_ROW = '50'


def load_data_from_excel(path: Path) -> Dict[str, Dict[str, float]]:
    """
    Reads the spreadsheet following the convention introduced in `create_excel_from_results.calculate_sheet` script
    """

    # NOTE: data_only allows to read values instead of formulas
    wb = load_workbook(filename = str(path), data_only=True)
    # access specific sheet
    sheet = wb[SHEET_NAME]
    # upper left cell - cell that initiates results table
    col_init = RESULT_INIT_COL
    row_init = RESULT_INIT_ROW

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
        # print(f"Checking sheet at {make_sheet_cell(col=col_iter, row=row_init)} for planner {planner_name}")
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
                    f"If they aren't, try to open the sheet at '{path}' and simply save it using Excel "
                    f"or LibreOffice Calc."
                )
            # collect
            metric_names.append(str(metric_name))
            metric_values.append(float(metric_value))
            # print(f"Checking metric ID {metric_names[-1]}, val {metric_values[-1]}")
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
