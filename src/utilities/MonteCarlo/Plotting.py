# Standard Imports
import os
import re

# External Imports
from bokeh.palettes import RdYlBu9


# Internal Imports
from Basilisk.utilities.DS_Plot import DS_Plot
from Basilisk.utilities import macros
from Basilisk.utilities.MonteCarlo.AnalysisBaseClass import mcAnalysisBaseClass
from Basilisk.utilities.dataframe_utilities import curve_per_df_column

class Plotting:
    """
    The class is used to automate many of things needed to be done to display
    """
    def __init__(self, dataDir, xAxisLabel='Time[ns]'):
        self.columns = 1 # Number of columns in the panel view to display
        self.dataDir = dataDir # Directory to pull the data from
        self.plots = dict[str, DS_Plot]() # a dictionary of DS_Plots
        self.varNames = list[str]()
        self.xAxisLabel = xAxisLabel

        self.CollectLogs(dataDir)


    def CollectLogs(self, path):
        """
        This function should parse the files in data_folder to find which variables are open to us as options
        :return:
        """
        if path is None:
            print("Please pass in a path to a data directory")

        # Check if folder exists
        if os.path.exists(path) is not True:
            print("Please pass in a valid path to a data directory")


        # Check if folder has runs in it
        if os.path.exists(path + "/run0.data") is not True:
            print("Data Directory must have at least one valid run in it.")

        # Grab variable names
        files = os.listdir(path)
        filterOut = re.compile('^((run)|(MonteCarlo)|(\.json))')

        # Get only the variable .data files and remove the .data extension
        # This WILL break if the output from MCs changes format.
        filtered_files = [ file[:-5] for file in files if not filterOut.match(file)]


        self.varNames = filtered_files


    def PlotVariable(self, varName:str):
        """
        Stage a specific variable to be plotted using default settings
        :param varName: The name of the variable or data file without .data
        :return:
        """


        plot = DS_Plot(varName, title=varName,
                xAxisLabel=self.xAxisLabel, yAxisLabel='Eclipse Factor',
                macro_x=macros.NANO2SEC, macro_y=macros.R2D,
                cmap=RdYlBu9,
                plotFcn=curve_per_df_column)

        self.plots[varName] = plot

    def PlotAllVariables(self):
        """
        Stage all collected data to be plotted
        :return:
        """
        for varName in self.varNames:
            self.PlotVariable(varName)


    def PlotExtremaRuns(self, varName, numExtrema, window):
        """
        Re pull the data from the data files for the variable and stage them to be plotted in extrema

        Things to think about:
            Once we find the extrema of one variable we will have a list of runs, do we want to display the plots of the other variables in those runs as well?
            Do we want to calculate the extrema runs of each
        :param varName: The variable to plot extrema runs for
        :param numExtrema: number of runs to find
        :param window: window of time to calculate extrema runs on
        :return:
        """
        pass

    def ShowStagedPlots(self):
        """
        Take all staged plots and return a panel
        :return:
        """

        analysis = mcAnalysisBaseClass()

        plotList = list(self.plots.values())

        return analysis.renderPlots(plotList)
