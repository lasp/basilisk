import holoviews as hv
from AnalysisBaseClass import mcAnalysisBaseClass
from Basilisk.utilities.datashader_utilities import DS_Plot, curve_per_df_column
from Basilisk.utilities import macros
from datashader.colors import Sets1to3


class MCAnalysisPlottingTools:
    """
    The main data structure in this class being manipulated is a dictionary of DS_Plots
    """
    def __init__(self, data_dir="", att_guid_name="attGuidMsg"):
        # key
        self.variables = list[str]()
        self.plots = dict[str, DS_Plot]()
        self.extremaPlots = dict[str, DS_Plot]()
        self.analysis = mcAnalysisBaseClass(data_dir)
        self.attGuidName = att_guid_name

    def render_plots(self):
        """
        This function should take all the collected mcAnalysisBaseClass objects, plot them,
        organize them into a layout and return pn.panel(layout).servable()
        similar to how AnalysisBaseClass renderPlots does it
        :return:
        """
        figures = []

        for variable in self.variables:
            figures.append(self.plots[variable].generateOverlay())

            if self.extremaPlots[variable] is not None:
                figures.append(self.extremaPlots[variable].generateOverlay())

        pass

    def add_plot(self, variable_name, title="", x_axis_label="x", y_axis_label="",
                 macro_x=macros.NANO2SEC, macro_y=1.0, labels=[], cmap=Sets1to3):
        """
        This function should take in the necessary information for a DS_Plot object
        along with a path to a data directory to add a plot to the list of plots

        :return:
        """
        data = self.analysis.pull_and_format_df(self.attGuidName + "." + variable_name)

        if title == "":
            title = variable_name

        if y_axis_label == "":
            y_axis_label = variable_name

        plot = DS_Plot(data, title=title, xAxisLabel=x_axis_label, yAxisLabel=y_axis_label, plotObjType=hv.Points,
                       labels=labels, macro_x=macro_x, macro_y=macro_y, cmap=cmap, plotFcn=curve_per_df_column)

        self.plots[variable_name] = plot
        self.variables.append(variable_name)

    def add_extrema_plot(self, variable_name):
        """
        This function should call getExtremaRunIndices and extractSubsetOfRuns to add a plot to the list
        labeled as an extrema plot
        This graph should directly display the curves without rastering them so that hover tools still work
        :return:
        """


        #need to change window
        extreme_run_numbers = self.analysis.getExtremaRunIndices(numExtrema=10, window=[1e9, 2e9])
