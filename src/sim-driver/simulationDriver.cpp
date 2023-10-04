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
        std::cout << nextStopTime << std::endl;
        while (this->sim.NextTaskTime <= this->stopTime && !this->terminate) {
            std::cout << nextStopTime << std::endl;
            std::cout << nextPriority << std::endl;
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

    /*
        This function is responsible for passing on the logger to a module instance (model), adding the
        model to a particular task, and defining
        the order/priority that the model gets updated within the task.

        :param TaskName (str): Name of the task
        :param NewModel (obj): Model to add to the task
        :param ModelData: None or struct containing, only used for C BSK modules
        :param ModelPriority (int): Priority that determines when the model gets updated. (Higher number = Higher priority)
    */
//    void SimulationDriver::addModelToTask(TaskName, NewModel, ModelData=None, ModelPriority=-1):
//        for Task in self.TaskList:
//            if Task.Name == TaskName:
//                Task.TaskData.AddNewObject(NewModel, ModelPriority)
//                TaskReplaceTag = 'self.TaskList[' + str(i) + ']'
//                TaskReplaceTag += '.TaskModels[' + str(len(Task.TaskModels)) + ']'
//                self.NameReplace[TaskReplaceTag] = NewModel.ModelTag
//                if ModelData is not None:
//                    try:
//                        ModelData.bskLogger = self.bskLogger
//                    except:
//                        pass
//                    Task.TaskModels.append(ModelData)
//                    self.simModules.add(inspect.getmodule(ModelData))
//                else:
//                    try:
//                        NewModel.bskLogger = self.bskLogger
//                    except:
//                        pass
//                    Task.TaskModels.append(NewModel)
//                    self.simModules.add(inspect.getmodule(NewModel))
//                return
//            i += 1
//        print("Could not find a Task with name: %(TaskName)s" % \
//              {"TaskName": TaskName})

    void SimulationDriver::setStopTime(uint64_t time) {
        this->stopTime = time;
    }

} // SimulationDriver