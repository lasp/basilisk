import numpy as np

from Basilisk.utilities.pythonVariableLogger import PythonVariableLogger
from Basilisk.utilities import macros
from Basilisk.utilities import SimulationBaseClass

from Basilisk.utilities.tests.TickerModule import TickerModule


def test_constructor():

    logger = PythonVariableLogger({
        "ticks": lambda CurrentSimNanos : CurrentSimNanos
    }, 1)

    assert logger.min_log_period is 1, "Failed to set min_log_period"
    assert logger.logging_functions["ticks"](10) is 10, "Failed to call logging function"



def test_Reset():
    logger = PythonVariableLogger({
        "ticks": lambda CurrentSimNanos : CurrentSimNanos
    }, 1)


    # arbitrarily set times
    logger._times = [0, 1, 2, 3, 4, 5]

    logger.Reset(5)

    assert len(logger._times) is 0, "Reset failed to clear times"

def test_times():
    logger = PythonVariableLogger({}, 1)

    logger._times = [0, 1, 2, 3, 4, 5]

    np.testing.assert_array_equal(logger.times(), np.array([0, 1, 2, 3, 4, 5]), "Fetched times correctly")


def test_logging():
    __tracebackhide__ = True

    simulation = SimulationBaseClass.SimBaseClass()
    simTime = 2.0

    process = simulation.CreateNewProcess("testProcess")

    task1Name = "task1"
    process.addTask(simulation.CreateNewTask(task1Name, macros.sec2nano(1.0)))

    testModule = TickerModule()
    testModule.ModelTag = "helloworldModule"
    simulation.AddModelToTask(task1Name, testModule)

    testLogger = PythonVariableLogger({"ticker": lambda _: testModule.GetTicker()})
    testLoggerHalf = PythonVariableLogger({"ticker": lambda _: testModule.GetTicker()}, 2)

    simulation.AddModelToTask(task1Name, testLogger)

    simulation.InitializeSimulation()

    simulation.ConfigureStopTime(macros.sec2nano(simTime))
    simulation.ExecuteSimulation()

    data = testLogger.GetData("ticker")

    assert data.shape[1] is 2, "Data does not contain two columns"
    assert data.shape[0] is int(simTime+1), "Data does not contain three rows"

    assert len(testLogger.times()) is 3
    np.testing.assert_array_equal(testLogger.ticker, np.array([1, 2, 3]), "Fetched values correctly")
