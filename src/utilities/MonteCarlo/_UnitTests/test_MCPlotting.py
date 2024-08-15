import unittest
from Basilisk.utilities.MonteCarlo.Plotting import Plotting


class MyTestCase(unittest.TestCase):
    def test_constructor(self):
        plotting = Plotting("tmp_montecarlo_test")

        assert plotting.dataDir is not None, "Data directory not correctly set"

        assert plotting.varNames is not None, "Did not find any variables"

        plotting.PlotAllVariables()

        assert plotting.plots, "plots not empty"


if __name__ == '__main__':
    unittest.main()
