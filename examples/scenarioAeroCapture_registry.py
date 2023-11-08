import sys

# append current directory so that python can find local modules
sys.path.append(".")

from src.utilities.registry import Registry

# The path to the location of Basilisk
# Used to get the location of supporting data.

import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.simulation import dragDynamicEffector
# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import tabularAtmosphere, simpleNav
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import vizSupport
from Basilisk.utilities.readAtmTable import readAtmTable

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

reg = Registry()

def sph2rv(xxsph):
    """
    NOTE: this function assumes inertial and planet-fixed frames are aligned
    at this time
    """
    
    r = xxsph[0]
    lon = xxsph[1]
    lat = xxsph[2]
    u = xxsph[3]
    gam = xxsph[4]
    hda = xxsph[5]
    
    NI = np.eye(3)
    IE = np.array([[np.cos(lat) * np.cos(lon), -np.sin(lon), -np.sin(lat) * np.cos(lon)],
                   [np.cos(lat) * np.sin(lon), np.cos(lon), -np.sin(lat) * np.sin(lon)],
                   [np.sin(lat), 0, np.cos(lat)]])
    ES = np.array([[np.cos(gam), 0, np.sin(gam)],
                   [-np.sin(gam) * np.sin(hda), np.cos(hda), np.cos(gam) * np.sin(hda)],
                   [-np.sin(gam) * np.cos(hda), -np.sin(hda), np.cos(gam) * np.cos(hda)]])
    
    e1_E = np.array([1,0,0])
    rvec_N = (r * NI @ IE) @ e1_E
    
    s3_S = np.array([0,0,1])
    uvec_N = u * ( NI @ IE @ ES) @ s3_S
    
    return rvec_N, uvec_N

def register():
    reg.register_model(model=simIncludeGravBody.gravBodyFactory, name="gravFactory")
    reg.register_model(model=dragDynamicEffector.DragDynamicEffector, name="dragEffector")
    reg.register_model(model=tabularAtmosphere.TabularAtmosphere, name="tabAtmo")
    reg.register_model(model=spacecraft.Spacecraft, name="scObject")
    reg.register_model(model=simpleNav.SimpleNav, name="simpleNavObj")
    # simpleNavObj.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    reg.register_message(source_name="scObject", target_name="simpleNavObj", message_data=("scStateOutMsg", "scStateInMsg"))
    # dragEffector.atmoDensInMsg.subscribeTo(tabAtmo.envOutMsgs[0])
    # reg.register_message(source_name="tabAtmo", target_name="dragEffector", message_data=("envOutMsgs", "atmoDensInMsg"))

def run(show_plots, planetCase):
    # This:
    #   * instantiates all models registered in the register() function
    #   * subscribes all messages registered above
    #   * returns a dict keyed by model name of the instantiated models.
    models = reg.init_models()

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    tabAtmo = models["tabAtmo"]
    tabAtmo.ModelTag = "tabularAtmosphere"            # update python name of test module
    atmoTaskName = "atmosphere"

    # define constants & load data
    if planetCase == 'Earth':
        r_eq = 6378136.6
        dataFileName = bskPath + '/supportData/AtmosphereData/EarthGRAMNominal.txt'
        altList, rhoList, tempList = readAtmTable(dataFileName, 'EarthGRAM')
    else:
        r_eq = 3397.2 * 1000
        dataFileName = bskPath + '/supportData/AtmosphereData/MarsGRAMNominal.txt'
        altList, rhoList, tempList = readAtmTable(dataFileName, 'MarsGRAM')

    # assign constants & ref. data to module
    tabAtmo.planetRadius = r_eq
    tabAtmo.altList = tabularAtmosphere.DoubleVector(altList)    
    tabAtmo.rhoList = tabularAtmosphere.DoubleVector(rhoList)
    tabAtmo.tempList = tabularAtmosphere.DoubleVector(tempList)

    # Drag Effector
    projArea = 10.0  # Set drag area in m^2
    dragCoeff = 2.2  # Set drag ceofficient
    m_sc = 2530.0    # kg

    dragEffector = models["dragEffector"]
    dragEffector.ModelTag = "DragEff"

    dragEffectorTaskName = "drag"
    dragEffector.coreParams.projectedArea = projArea
    dragEffector.coreParams.dragCoeff = dragCoeff
    dragEffector.coreParams.comOffset = [1., 0., 0.]

    dynProcess.addTask(scSim.CreateNewTask(atmoTaskName, simulationTimeStep))
    dynProcess.addTask(scSim.CreateNewTask(dragEffectorTaskName, simulationTimeStep))
    scSim.AddModelToTask(atmoTaskName, tabAtmo)

    # Add test module to runtime call list
    scSim.AddModelToTask(simTaskName, tabAtmo)

    scObject = models["scObject"]
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = m_sc
    tabAtmo.addSpacecraftToModel(scObject.scStateOutMsg)

    simpleNavObj = models["simpleNavObj"]
    scSim.AddModelToTask(simTaskName, simpleNavObj)
    # simpleNavObj.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    scObject.addDynamicEffector(dragEffector)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(dragEffectorTaskName, dragEffector)
    # clear prior gravitational body and SPICE setup definitions

    # breakpoint()
    dragEffector.atmoDensInMsg.subscribeTo(tabAtmo.envOutMsgs[0])

    gravFactory = models["gravFactory"]
    if planetCase == 'Earth':
        planet = gravFactory.createEarth()
    else:
        planet = gravFactory.createMars()
    planet.isCentralBody = True  # ensure this is the central gravitational body
    mu = planet.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    if planetCase == 'Earth':
        r = 6503 * 1000
        u = 11.2 * 1000
        gam = -5.15 * macros.D2R
    else:
        r = (3397.2 + 125.) * 1000
        u = 6 * 1000
        gam = -10 * macros.D2R
    lon = 0
    lat = 0
    hda = np.pi/2
    xxsph = [r,lon,lat,u,gam,hda]
    rN, vN = sph2rv(xxsph)
    
    scObject.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m - v_CN_N

    # set the simulation time
    if planetCase == 'Earth':
        simulationTime = macros.sec2nano(300)
    else:
        simulationTime = macros.sec2nano(400)

    #
    #   Setup data logging before the simulation is initialized
    #

    dataLog = scObject.scStateOutMsg.recorder()
    dataNewAtmoLog = tabAtmo.envOutMsgs[0].recorder()
    scSim.AddModelToTask(simTaskName, dataLog)
    scSim.AddModelToTask(simTaskName, dataNewAtmoLog)

    #
    #   initialize Spacecraft States with initialization variables
    #
    scObject.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m - v_CN_N

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                        # , saveFile=fileName
                                        )
    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = dataLog.r_BN_N
    velData = dataLog.v_BN_N
    densData = dataNewAtmoLog.neutralDensity
    np.set_printoptions(precision=16)

    figureList = {}
    plt.close("all")  # clears out plots from earlier test runs

    # draw the inertial position vector components
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(0,3):
        plt.plot(dataLog.times()*macros.NANO2MIN, posData[:, idx]/1000.,
                 color=unitTestSupport.getLineColor(idx,3),
                 label='$r_{BN,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Inertial Position [km]')

    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    smaData = []
    engData = []
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem(mu, posData[idx, 0:3], velData[idx, 0:3])
        smaData.append(oeData.a/1000.)
        engData.append(-mu/(2*oeData.a)/1e6)    # km^2/s^2
    plt.plot(dataLog.times()*macros.NANO2MIN, engData
             , color='#aa0000'
             )
    plt.xlabel('Time [min]')
    plt.ylabel('Energy [km^2/s^2]')
    plt.grid()
    pltName = fileName + "2" + planetCase
    figureList[pltName] = plt.figure(2)

    r = np.linalg.norm(posData, axis=1)
    v = np.linalg.norm(velData, axis=1)

    plt.figure(3)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    plt.plot(dataNewAtmoLog.times()*macros.NANO2MIN, densData)
    plt.xlabel('Time [min]')
    plt.ylabel('Density in kg/m^3')
    pltName = fileName + "3" + planetCase
    figureList[pltName] = plt.figure(3)

    plt.figure(4)
    fig = plt.gcf()
    ax = fig.gca()
    plt.plot(v/1e3, (r-r_eq)/1e3)
    plt.xlabel('velocity [km/s]')
    plt.ylabel('altitude [km]')
    plt.grid()
    pltName = fileName + "4" + planetCase
    figureList[pltName] = plt.figure(4)

    plt.figure(5)
    fig = plt.gcf()
    ax = fig.gca()
    plt.plot(dataLog.times()*macros.NANO2MIN, (r-r_eq)/1e3)
    plt.xlabel('time [min]')
    plt.ylabel('altitude [km]')
    plt.grid()
    pltName = fileName + "5" + planetCase
    figureList[pltName] = plt.figure(5)

    if show_plots:
        plt.show()
        plt.close("all")

    return figureList


if __name__ == "__main__":
    # From the user's point of view, not very much changes except that you can now
    # register all the modules first, and then initialize them at the beginning of the run.
    # Then to access the models, you can use the dictionary returned from the initialization.
    # NOTE: registering and initializing is not enforced, rather is optional. However, if you
    #   do not register before instantiating everything then there will be no access to the
    #   messaging graph that is generated through the singleton.
    register()
    run(True, 'Mars')