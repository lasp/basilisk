//
// Created by Patrick Kenneally on 9/26/23.
//

#include "simulationDriver.h"

#include <utility>

namespace SimulationDriver {
    SimulationDriver::SimulationDriver() :sim() {
    }

    void SimulationDriver::run() {
        this->initializeSimulation();
        this->executeSimulation();
    }

    void SimulationDriver::initializeSimulation() {
        if (this->simulationInitialized) {
            this->sim.resetThreads(this->sim.getThreadCount());
        }
        this->sim.assignRemainingProcs();
        this->sim.ResetSimulation();
        this->sim.selfInitSimulation();

        this->sim.resetInitSimulation();

        this->simulationInitialized = true;
    }

    void SimulationDriver::executeSimulation() {
        uint64_t nextStopTime = this->sim.NextTaskTime;
        int nextPriority = -1;
        while (this->sim.NextTaskTime <= this->stopTime && !this->terminate) {
            this->sim.StepUntilStop(nextStopTime, nextPriority);
            nextPriority = -1;
            nextStopTime = this->stopTime;
        }
    }


//    Creates a process and adds it to the sim
//
//    :param procName (str): Name of process
//    :param priority (int): Priority that determines when the model gets updated. (Higher number = Higher priority)
//    :return: simulationArchTypes.ProcessBaseClass object
    std::unique_ptr<SysProcess> SimulationDriver::createProcess(std::string name, int priority = -1) {
        auto process = std::make_unique<SysProcess>();
        process->setProcessName(std::move(name));
        process->setPriority(priority);
        this->sim.addNewProcess(process.get());
        return process;
    }

    /*
        Creates a simulation task on the C-level with a specific update-frequency (TaskRate), an optional delay, and
        an optional start time.

        :param TaskName (str): Name of Task
        :param TaskRate (int): Number of nanoseconds to elapse before update() is called
        :param InputDelay (int): Number of nanoseconds simulating a lag of the particular task# TODO: Check that this is [ns]
        :param FirstStart (int): Number of nanoseconds to elapse before task is officially enabled
        :return: simulationArchTypes.TaskBaseClass object
    */
    std::unique_ptr<SysModelTask> SimulationDriver::createTask(std::string name,
                                                               uint64_t rate,
                                                               uint64_t inputDelay,
                                                               uint64_t firstStart) {
        auto task = std::make_unique<SysModelTask>(rate, inputDelay,firstStart);
        task->TaskName = std::move(name);
        return task;
    }

    void SimulationDriver::setStopTime(uint64_t time) {
        this->stopTime = time;
    }

} // SimulationDriver