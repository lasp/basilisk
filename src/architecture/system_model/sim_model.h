/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */
#ifndef _SimModel_HH_
#define _SimModel_HH_

#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/bskSemaphore.h"
#include "architecture/system_model/sys_process.h"
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdint.h>
#include <set>
#include <thread>
#include <vector>

//! This class handles the management of a given "thread" of execution and provides the main mechanism for running concurrent jobs inside BSK
class SimThreadExecution
{
public:
    SimThreadExecution()=default;
    explicit SimThreadExecution(uint64_t threadIdent, uint64_t currentSimNanos=0);
    ~SimThreadExecution()=default;
    void updateNewStopTime(uint64_t newStopNanos) {stopThreadNanos = newStopNanos;}  //!< Update simulation stop time
    void clearProcessList() {processList.clear();}  //!< Clear the process list
    void selfInitProcesses() const;
    void crossInitProcesses() const;
    void resetProcesses();
    void addNewProcess(SysProcess* newProc);
    uint64_t procCount() const {return processList.size();} //!< Gets the current "thread-count" in the system
    bool threadActive() const {return this->threadRunning;} //!< Is the thread is currently allocated processes and is in execution
    void threadReady() {this->threadRunning=true;} //!< Put the thread into a running state
    void waitOnInit();
    void postInit();
    bool threadValid() const {return !this->terminateThread;} //!< Is the thread currently usable or if it has been requested to shutdown
    void killThread() {this->terminateThread=true;} //!< Asks the thread to no longer be alive
    void lockThread();
    void unlockThread();
    void lockParent();
    void unlockParent();
    void StepUntilStop();  //!< Step simulation until stop time reached
    void SingleStepProcesses(int64_t stopPri=-1); //!< Step only the next task in the simulation
    void moveProcessMessages() const;
    uint64_t getCurrentNanos() const;
    void setCurrentNanos(uint64_t currentNanos);
    uint64_t getNextTaskTime() const;
    void setNextTaskTime(uint64_t nextTaskTime);
    uint64_t getCurrentThreadNanos() const;
    uint64_t getStopThreadNanos() const;
    void setStopThreadNanos(uint64_t stopThreadNanos);

    int64_t stopThreadPriority=-1; //!< Current stop priority for thread
    uint64_t threadID=0;          //!< Identifier for thread
    std::thread *threadContext=nullptr; //!< std::thread data for concurrent execution
    int64_t nextProcPriority=-1;  //!< [-] Priority level for the next process
    bool selfInitNow{};              //!< Flag requesting self init
    bool crossInitNow{};             //!< Flag requesting cross-init
    bool resetNow{};                 //!< Flag requesting that the thread execute reset
private:
    uint64_t currentThreadNanos=0;  //!< Current simulation time available at thread
    uint64_t stopThreadNanos=0;   //!< Current stop conditions for the thread
    uint64_t CurrentNanos=0;  //!< [ns] Current sim time
    uint64_t NextTaskTime=0;  //!< [ns] time for the next Task
    bool threadRunning{};            //!< Flag that will allow for easy concurrent locking
    bool terminateThread{};          //!< Flag that indicates that it is time to take thread down
    BSKSemaphore parentThreadLock;   //!< Lock that ensures parent thread won't proceed
    BSKSemaphore selfThreadLock;     //!< Lock that ensures this thread only reaches allowed time
    std::vector<SysProcess*> processList;  //!< List of processes associated with thread
    std::mutex initReadyLock;      //!< Lock function to ensure runtime locks are configured
    std::condition_variable initHoldVar; //!< Conditional variable used to prevent race conditions
};

//! The top-level container for an entire simulation
class SimModel
{
public:
    SimModel();
    ~SimModel();

    void selfInitSimulation();  //!< Method to initialize all added Tasks
    void resetInitSimulation() const;  //!< Method to reset all added tasks
    void StepUntilStop(uint64_t SimStopTime, int64_t stopPri);  //!< Step simulation until stop time uint64_t reached
    void SingleStepProcesses(int64_t stopPri=-1); //!< Step only the next Task in the simulation
    void addNewProcess(SysProcess *newProc);
    void addProcessToThread(SysProcess *newProc, uint64_t threadSel);
    void ResetSimulation();  //!< Reset simulation back to zero
    void clearProcsFromThreads() const;
    void resetThreads(uint64_t threadCount);
    void deleteThreads();
    void assignRemainingProcs();
    uint64_t getThreadCount() const {return threadList.size();} //!< returns the number of threads used
    uint64_t getCurrentNanos() const;
    uint64_t getNextTaskTime() const;
    BSKLogger bskLogger;
    std::vector<SysProcess *> processList;  //!< -- List of processes we've created
    std::vector<SimThreadExecution*> threadList{};  //!< -- Array of threads that we're running on
    std::string SimulationName;  //!< -- Identifier for Sim
    int64_t nextProcPriority=-1;  //!< [-] Priority level for the next process

private:
    uint64_t CurrentNanos=0;  //!< [ns] Current sim time
    uint64_t NextTaskTime=0;  //!< [ns] time for the next Task
};

#endif /* _SimModel_H_ */
