//
// Created by Patrick Kenneally on 9/26/23.
//

#ifndef LASP_BASILISK_REDUX_SIMULATIONDRIVER_H
#define LASP_BASILISK_REDUX_SIMULATIONDRIVER_H

#include "../architecture/system_model/sim_model.h"
#include "../architecture/system_model/sys_model_task.h"
#include "../architecture/system_model/sys_process.h"

namespace SimulationDriver {

    class SimulationDriver {
    public:
        SimulationDriver();
        ~SimulationDriver() = default;

        void setStopTime(uint64_t time);
        void initializeSimulation();
        void run();
        std::unique_ptr<SysProcess> createProcess(std::string name, int priority);
        std::unique_ptr<SysModelTask> createTask(std::string name,
                                                 uint64_t rate,
                                                 uint64_t inputDelay=0,
                                                 uint64_t firstStart=0);

    private:
        void executeSimulation();

        SimModel sim;
        bool simulationInitialized;
        uint64_t stopTime = 0;
        bool terminate = false;
    };

} // SimulationDriver

#endif //LASP_BASILISK_REDUX_SIMULATIONDRIVER_H
