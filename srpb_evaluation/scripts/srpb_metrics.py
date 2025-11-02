class SrpbMetrics:
    def __init__(self) -> None:
        self.tex_missing_metric_value = r"---"
        # short names to avoid line breaking
        # excel_fun_range_begin
        self.efrb = "{#1}"
        # excel_fun_range_end
        self.efre = "{#2}"

        self.map =  {
            's_rbt': {
                'tex_name': r"$m_{\mathrm{srbt}}$",
                'tex_unit': r"$\left[ \mathrm{unit} \right]$",
                'description': 'Mean number of robot samples collected during a single trial',
                # not necessarily true, but less samples with the same metric results indicates better effectiveness
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            's_ppl': {
                'tex_name': r"$m_{\mathrm{sppl}}$",
                'tex_unit': r"$\left[ \mathrm{unit} \right]$",
                'description': 'Mean number of people samples collected during a single trial',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            's_grp': {
                'tex_name': r"$m_{\mathrm{sgrp}}$",
                'tex_unit': r"$\left[ \mathrm{unit} \right]$",
                'description': 'Mean number of group samples collected during a single trial',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },

            'm_goal':  {
                'tex_name': r"$m_{\mathrm{goal}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Navigation success rate',
                'min_is_best': False,
                'excel_calc_fun': f'=100.0 * (SUM({self.efrb}:{self.efre}) / COLUMNS({self.efrb}:{self.efre}))'
            },
            # for the backward compatibility with IEEE Access article, this metric (not presented in the paper)
            # is named '_dist m_obs', whereas violations metric (m_obs_viol) is 'm_obs' (as presented)
            'm_obs':  {
                'tex_name': r"${}_{\mathrm{dist}} m_{\mathrm{obs}}$",
                'tex_unit': r"$\left[ \mathrm{m} \right]$",
                'description': 'Mean distance to the closest obstacle',
                'min_is_best': False,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_obs_min':  {
                'tex_name': r"${}_{\mathrm{min}} m_{\mathrm{obs}}$",
                'tex_unit': r"$\left[ \mathrm{m} \right]$",
                'description': 'Minimum distance to the closest obstacle',
                'min_is_best': False,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_obs_max':  {
                'tex_name': r"${}_{\mathrm{max}} m_{\mathrm{obs}}$",
                'tex_unit': r"$\left[ \mathrm{m} \right]$",
                'description': 'Maximum distance to the closest obstacle',
                'min_is_best': False,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_obs_viol':  {
                'tex_name': r"$m_{\mathrm{obs}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Obstacle safety',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_mef':  {
                'tex_name': r"$m_{\mathrm{mef}}$",
                'tex_unit': r"$\left[ \mathrm{s} \right]$",
                'description': 'Motion efficiency',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_path': {
                'tex_name': r"$m_{\mathrm{plin}}$",
                'tex_unit': r"$\left[ \mathrm{m} \right]$",
                'description': 'Path length',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_chc':  {
                'tex_name': r"$m_{\mathrm{chc}}$",
                'tex_unit': r"$\left[ \mathrm{rad} \right]$",
                'description': 'Cumulative heading change',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_cef':  {
                'tex_name': r"$m_{\mathrm{cef}}$",
                'tex_unit': r"$\left[ 10^{-3} \cdot \mathrm{s} \right]$",
                'description': 'Computational efficiency',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_cre':  {
                'tex_name': r"$m_{\mathrm{cre}}$",
                'tex_unit': r"$\left[ 10^{-3} \cdot \mathrm{s} \right]$",
                'description': 'Computational time repeatability',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_vsm':  {
                'tex_name': r"$m_{\mathrm{vsm}}$",
                'tex_unit': r"$\left[ \mathrm{\frac{m}{s^2}} \right]$",
                'description': 'Velocity smoothness',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_hsm':  {
                'tex_name': r"$m_{\mathrm{hsm}}$",
                'tex_unit': r"$\left[ \mathrm{\frac{rad}{s^2}} \right]$",
                'description': 'Heading change smoothness',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_osc':  {
                'tex_name': r"$m_{\mathrm{osc}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Oscillations',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_bwd':  {
                'tex_name': r"$m_{\mathrm{bwd}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Backward movements',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_inp':  {
                'tex_name': r"$m_{\mathrm{iprot}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'In-place rotations',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_psi':  {
                'tex_name': r"$m_{\mathrm{psi}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Personal spaces intrusion',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_psi_min':  {
                'tex_name': r"${}_{\mathrm{min}} m_{\mathrm{psi}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Minimum value of the personal spaces intrusion',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_psi_max':  {
                'tex_name': r"${}_{\mathrm{max}} m_{\mathrm{psi}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Maximum value of the personal spaces intrusion',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_psi_viol': {
                'tex_name': r"${}_{\mathrm{viol}} m_{\mathrm{psi}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Percentage of the personal spaces intrusion violations',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_fsi':  {
                'tex_name': r"$m_{\mathrm{fsi}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'F-Formations\' O-spaces intrusion',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_fsi_min':  {
                'tex_name': r"${}_{\mathrm{min}} m_{\mathrm{fsi}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Minimum value of the F-Formations\' O-spaces intrusion',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_fsi_max':  {
                'tex_name': r"${}_{\mathrm{max}} m_{\mathrm{fsi}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Maximum value of the F-Formations\' O-spaces intrusion',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_fsi_viol': {
                'tex_name': r"${}_{\mathrm{viol}} m_{\mathrm{fsi}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Percentage of the F-Formations\' O-spaces intrusion violations',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_dir':  {
                'tex_name': r"$m_{\mathrm{dir}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Heading direction discomfort',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_dir_min':  {
                'tex_name': r"${}_{\mathrm{min}} m_{\mathrm{dir}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Minimum value of the heading direction discomfort',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_dir_max':  {
                'tex_name': r"${}_{\mathrm{max}} m_{\mathrm{dir}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Maximum value of the heading direction discomfort',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_dir_viol': {
                'tex_name': r"${}_{\mathrm{viol}} m_{\mathrm{dir}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Percentage of the heading direction discomfort violations',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_psd':  {
                'tex_name': r"$m_{\mathrm{psd}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Passing speed discomfort',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_psd_min':  {
                'tex_name': r"${}_{\mathrm{min}} m_{\mathrm{psd}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Minimum value of a passing speed discomfort',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_psd_max':  {
                'tex_name': r"${}_{\mathrm{max}} m_{\mathrm{psd}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Maximum value of a passing speed discomfort',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            },
            'm_psd_viol': {
                'tex_name': r"${}_{\mathrm{viol}} m_{\mathrm{psd}}$",
                'tex_unit': r"$\left[ \% \right]$",
                'description': 'Percentage of the passing speed discomfort violations',
                'min_is_best': True,
                'excel_calc_fun': f'=MEDIAN({self.efrb}:{self.efre})'
            }
        }


    def get(self) -> dict:
        """
        Returns a whole map
        """
        return self.map


    def has_metric(self, metric_id: str) -> bool:
        """
        Evaluates whether a given metric exists
        """
        return metric_id in self.map.keys()


    def get_metric(self, metric_id: str) -> dict:
        """
        Returns all data related to a certain metric_id
        """
        if not self.has_metric(metric_id):
            Exception(f'No such metric_id as {metric_id}')
        return self.map[metric_id]


    def get_metrics(self, metrics_id: list) -> dict:
        """
        Returns a dict with data related to selected metrics
        """
        metrics_map = {}
        for name in metrics_id:
            if not self.has_metric(name):
                raise Exception(
                    f"Could not find {name} in the SRPB metrics map. Available metric names are: {self.map.keys()}"
                )
            metrics_map[name] = self.get_metric(name)
        return metrics_map


    def get_latex_unit(self, metric_id: str) -> str:
        if not self.has_metric(metric_id):
            Exception(f'No such metric_id as {metric_id}')
        return self.map[metric_id]['tex_unit']


    def get_latex_name(self, metric_id: str) -> str:
        if not self.has_metric(metric_id):
            Exception(f'No such metric_id as {metric_id}')
        return self.map[metric_id]['tex_name']


    def get_description(self, metric_id: str) -> str:
        if not self.has_metric(metric_id):
            Exception(f'No such metric_id as {metric_id}')
        return self.map[metric_id]['description']


    def is_minimum_best(self, metric_id: str) -> bool:
        """
        Returns True if the minimum value of the metric is desired, or False otherwise
        """
        if not self.has_metric(metric_id):
            Exception(f'No such metric_id as {metric_id}')
        return self.map[metric_id]['min_is_best']


    def get_excel_calc_fun_raw(self, metric_id: str) -> str:
        """
        Returns the raw string representation of an Excel function
        """
        if not self.has_metric(metric_id):
            Exception(f'No such metric_id as {metric_id}')
        return self.map[metric_id]['excel_calc_fun']


    def get_excel_calc_fun(self, metric_id: str, range_begin: str, range_end: str) -> str:
        """
        Returns the string representation of an Excel function
        """
        fun_str = self.get_excel_calc_fun_raw(metric_id)
        fun_str = fun_str.replace(self.efrb, range_begin)
        fun_str = fun_str.replace(self.efre, range_end)
        return fun_str


    def get_latex_missing_metric_value(self) -> str:
        return self.tex_missing_metric_value
