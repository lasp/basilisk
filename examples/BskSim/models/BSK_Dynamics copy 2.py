#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

import numpy as np
from Basilisk import __path__
from Basilisk.simulation import ephemerisConverter
from Basilisk.simulation import (spacecraft, extForceTorque, simpleNav,
                                 reactionWheelStateEffector, coarseSunSensor, eclipse)
from Basilisk.simulation import thrusterDynamicEffector
from Basilisk.simulation import fuelTank
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros as mc
from Basilisk.utilities import simIncludeRW, simIncludeGravBody
from Basilisk.utilities import simIncludeThruster
from Basilisk.utilities import unitTestSupport as sp

bskPath = __path__[0]


class BSKDynamicModels():
    """
    General bskSim simulation class that sets up the spacecraft simulation configuration.

    """
    ##### "init" is a constructor 
    ##### "SimBase" and dynRate are constructor parameter
    
    def __init__(self, SimBase, dynRate):
        # define empty class variables
        self.sun = None
        self.earth = None
        self.moon = None
        self.epochMsg = None
        self.RW1 = None
        self.RW2 = None
        self.RW3 = None
        self.RW4 = None

        #### in here we are assigning "processName" attribute value to "DynamicsProcessName" which is an attribute of the class instace "SimBase"
        #### then we assign a string to the task name of same "self" class which is "BSKDynamicsModels"
        #### for 3rd attribute we initializes the "processTasksTimeStep" attribute with the result of mc.sec2nano(dynRate)
        #### where the function sec2nano is in a class named mc which convert the dynRate from sec to nanosec.

        # Define process name, task name and task time-step
        self.processName = SimBase.DynamicsProcessName
        self.taskName = "DynamicsTask"
        self.processTasksTimeStep = mc.sec2nano(dynRate)
        
        #####  here we create new task using a function named "CreateNewTask" in the "SimBase" object. The arguments passed to this method are "self.taskName and "self.processTasksTimeStep"

        # Create task
        SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep))

        ##### here we are instantiating an attribute which means creating an instance of it .. we can tell from the bruckets after the class name 
        #### "self.scObject" is an instance of the Spacecraft class which lives in spacecraft module 
        #### parentheses are used to call the class's constructor, which is the __init__ method
        #### constructor being called or these modules (elcipse, simplenav..etc) being created
        #### gives us blank canvas version of one of those modules and then we add things to it and configure it

        # Instantiate Dyn modules as objects
        self.scObject = spacecraft.Spacecraft()
        self.gravFactory = simIncludeGravBody.gravBodyFactory()
        self.rwFactory = simIncludeRW.rwFactory()
        self.extForceTorqueObject = extForceTorque.ExtForceTorque()
        self.simpleNavObject = simpleNav.SimpleNav()
        self.eclipseObject = eclipse.Eclipse()
        self.CSSConstellationObject = coarseSunSensor.CSSConstellation()
        self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
        self.thrustersDynamicEffectorACS = thrusterDynamicEffector.ThrusterDynamicEffector()
        self.thrustersDynamicEffectorDV = thrusterDynamicEffector.ThrusterDynamicEffector()
        self.EarthEphemObject = ephemerisConverter.EphemerisConverter()

        self.fuelTankStateEffector = fuelTank.FuelTank()
        self.fuelTankStateEffector.setTankModel(fuelTank.TANK_MODEL_CONSTANT_VOLUME)
        self.tankModel = fuelTank.cvar.FuelTankModelConstantVolume

        self.fuelTankStateEffector2 = fuelTank.FuelTank()
        self.fuelTankStateEffector2.setTankModel(fuelTank.TANK_MODEL_CONSTANT_DENSITY)
        self.tankModel2 = fuelTank.cvar.FuelTankModelConstantDensity



        # Initialize all modules and write init one-time messages
        self.InitAllDynObjects()

        # Assign initialized modules to tasks
        SimBase.AddModelToTask(self.taskName, self.scObject, 201)
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject, 109)
        SimBase.AddModelToTask(self.taskName, self.gravFactory.spiceObject, 200)
        SimBase.AddModelToTask(self.taskName, self.EarthEphemObject, 199)
        SimBase.AddModelToTask(self.taskName, self.CSSConstellationObject, 108)
        SimBase.AddModelToTask(self.taskName, self.eclipseObject, 204)
        SimBase.AddModelToTask(self.taskName, self.rwStateEffector, 301)
        SimBase.AddModelToTask(self.taskName, self.thrustersDynamicEffectorACS, 302)
        SimBase.AddModelToTask(self.taskName, self.thrustersDynamicEffectorDV, 303)
        SimBase.AddModelToTask(self.taskName, self.extForceTorqueObject, 300)
        SimBase.AddModelToTask(self.taskName, self.fuelTankStateEffector,200)
        SimBase.AddModelToTask(self.taskName, self.fuelTankStateEffector2,210)

        
        SimBase.createNewEvent("addOneTimeRWFault", self.processTasksTimeStep, True,
            ["self.TotalSim.CurrentNanos>=self.oneTimeFaultTime and self.oneTimeRWFaultFlag==1"],
            ["self.DynModels.AddRWFault('friction',0.05,1, self.TotalSim.CurrentNanos)", "self.oneTimeRWFaultFlag=0"])

        
        SimBase.createNewEvent("addRepeatedRWFault", self.processTasksTimeStep, True,
            ["self.repeatRWFaultFlag==1"],
            ["self.DynModels.PeriodicRWFault(1./3000,'friction',0.005,1, self.TotalSim.CurrentNanos)", "self.setEventActivity('addRepeatedRWFault',True)"])

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
        

    def SetSpacecraftHub(self):
        
        #Specify the spacecraft parameters.
        r_BcB_B = self.scObject.hub.r_BcB_B
        self.I_sc = [900., 0., 0.,
                 0., 800., 0.,
                 0., 0., 600.]
        self.scObject.hub.mHub = 750.0  # kg - spacecraft mass
        self.scObject.hub.r_BcB_B = np.array([[-10.0], [-3.0], [17.0]])  # m - position vector of body-fixed point B relative to CM
        self.scObject.hub.IHubPntBc_B = sp.np2EigenMatrix3d(self.I_sc)
        self.scObject.skew_matrix_r_BcB_B = np.array([[0, -r_BcB_B[2][0], r_BcB_B[1][0]],
                                                 [r_BcB_B[2][0], 0, -r_BcB_B[0][0]],
                                                 [-r_BcB_B[1][0], r_BcB_B[0][0], 0]])
        self.skew_matrix_r_BcB_B_transpose = self.scObject.skew_matrix_r_BcB_B.T
        self.I_sc_B = self.scObject.hub.IHubPntBc_B + self.scObject.hub.mHub * (np.dot(self.scObject.skew_matrix_r_BcB_B , self.skew_matrix_r_BcB_B_transpose))
        print("The value of self.I_sc_B is:",self.I_sc_B)        
        print(  "                      "  )
        

        #Specify fuel tank 1 parameters
        tankModel_r_t1c_B = np.array([[-3.0], [-1.0], [15.0]]) 
        r_t1c_B = tankModel_r_t1c_B      
        tankModel_mass_tank1 = 9                                                      
        tankModel_tank1_Radious = 7             
        tankModel_I_t1_t1c =[tankModel_mass_tank1*(tankModel_tank1_Radious)**2, 0., 0.,
                              0., tankModel_mass_tank1*(tankModel_tank1_Radious)**2, 0.,
                              0., 0., tankModel_mass_tank1*(tankModel_tank1_Radious)**2]       
        EtankModel_I_t1_t1c = sp.np2EigenMatrix3d(tankModel_I_t1_t1c)             
        print("The value of EtankModel_I_t1_t1c is:",EtankModel_I_t1_t1c)  
        print(  "                      "  )
        tankModel_skew_matrix_r_t1c_B = np.array([[0, -r_t1c_B[2][0], r_t1c_B[1][0]],
                                           [r_t1c_B[2][0], 0, -r_t1c_B[0][0]],
                                           [-r_t1c_B[1][0], r_t1c_B[0][0], 0]])
        skew_matrix_r_t1c_B_transpose = tankModel_skew_matrix_r_t1c_B.T
        tankModel_I_t1_B = EtankModel_I_t1_t1c + tankModel_mass_tank1* (np.dot(tankModel_skew_matrix_r_t1c_B , skew_matrix_r_t1c_B_transpose))
        print("The value of tankModel_I_t1_B is:",tankModel_I_t1_B)        
        print(  "                      "  )
  
        
        #Specify fuel tank 2 parameters
        tankModel2_r_t2c_B = np.array([[5.0], [2.0], [20.0]])            
        r_t2c_B = tankModel2_r_t2c_B
        tankModel2_mass_tank2 = 14                                                     
        tankModel2_tank2_Radious = 9                                             
        tankModel2_I_t2_t2c =[tankModel2_mass_tank2*(tankModel2_tank2_Radious)**2, 0., 0.,
                              0., tankModel2_mass_tank2*(tankModel2_tank2_Radious)**2, 0.,
                              0., 0., tankModel2_mass_tank2*(tankModel2_tank2_Radious)**2]   
        EtankModel_I_t2_t2c = sp.np2EigenMatrix3d(tankModel2_I_t2_t2c)
        print("The value of EtankModel_I_t2_t2c is:",EtankModel_I_t2_t2c)        
        print(  "                      "  )
        tankModel2_skew_matrix_r_t2c_B = np.array([[0, -r_t2c_B[2][0], r_t2c_B[1][0]],
                                                   [r_t2c_B[2][0], 0, -r_t2c_B[0][0]],
                                                   [-r_t2c_B[1][0], r_t2c_B[0][0], 0]])
        skew_matrix_r_t2c_B_transpose = tankModel2_skew_matrix_r_t2c_B.T
        tankModel2_I_t2_B = EtankModel_I_t2_t2c + tankModel2_mass_tank2 * (np.dot(tankModel2_skew_matrix_r_t2c_B , skew_matrix_r_t2c_B_transpose))
        print("The value of tankModel2_I_t2_B is:",tankModel2_I_t2_B)
        print(  "                      "  )

 
        """
        Calculate the spacecraft parameters without considering the fuel tanks.
        """
        mass__without_tanks = self.scObject.hub.mHub - tankModel_mass_tank1 - tankModel2_mass_tank2
        I_hub_B_without_tanks = self.I_sc_B - tankModel2_I_t2_B - tankModel_I_t1_B
        scObject_hub_r_Bc_B = ((self.scObject.hub.mHub * np.array(self.scObject.hub.r_BcB_B)) - (tankModel2_mass_tank2 * np.array(tankModel2_r_t2c_B)) - (tankModel_mass_tank1 * np.array(tankModel_r_t1c_B))) / mass__without_tanks

        #r_c_B_without_tanks = self.scObject.hub.r_c_B
        #I_sc_B_without_tanks = I_sc_B_without_tanks + mass_sc_without_tanks * np.dot(self.scObject.skew_matrix_r_c_B, self.scObject.skew_matrix_r_c_B.T)

        print("Mass of spacecraft without tanks (mass__without_tanks):\n", mass__without_tanks)
        print(  "                      \n"  ) 
        print("Inertia of spacecraft without tanks ( I_hub_B_without_tanks):\n", I_hub_B_without_tanks)
        print(  "                     \n"  ) 
        print("position of spacecraft hub without tanks (scObject_hub_r_Bc_B):\n", scObject_hub_r_Bc_B)
        print(  "                      \n"  ) 
        




    def SetGravityBodies(self):
        """
        Specify what gravitational bodies to include in the simulation
        """
        timeInitString = "2012 MAY 1 00:28:30.0"
        gravBodies = self.gravFactory.createBodies(['sun', 'earth', 'moon'])
        gravBodies['earth'].isCentralBody = True
        self.sun = 0
        self.earth = 1
        self.moon = 2

        self.gravFactory.addBodiesTo(self.scObject)
        self.gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                              timeInitString,
                                              epochInMsg=True)
        self.epochMsg = self.gravFactory.epochMsg

        self.gravFactory.spiceObject.zeroBase = 'Earth'

        self.EarthEphemObject.addSpiceInputMsg(self.gravFactory.spiceObject.planetStateOutMsgs[self.earth])

    def SetEclipseObject(self):
        """
        Specify what celestial object is causing an eclipse message.
        """
        self.eclipseObject.ModelTag = "eclipseObject"
        self.eclipseObject.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
        # add all celestial objects in spiceObjects except for the sun (0th object)
        for c in range(1, len(self.gravFactory.spiceObject.planetStateOutMsgs)):
            self.eclipseObject.addPlanetToModel(self.gravFactory.spiceObject.planetStateOutMsgs[c])
        self.eclipseObject.addSpacecraftToModel(self.scObject.scStateOutMsg)

    def SetExternalForceTorqueObject(self):
        """Set the external force and torque object."""
        self.extForceTorqueObject.ModelTag = "externalDisturbance"
        self.scObject.addDynamicEffector(self.extForceTorqueObject)

    def SetSimpleNavObject(self):
        """Set the navigation sensor object."""
        self.simpleNavObject.ModelTag = "SimpleNavigation"
        self.simpleNavObject.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)

    def SetReactionWheelDynEffector(self):
        """Set the 4 reaction wheel devices."""
        # specify RW momentum capacity
        maxRWMomentum = 50.  # Nms

        # Define orthogonal RW pyramid
        # -- Pointing directions
        rwElAngle = np.array([40.0, 40.0, 40.0, 40.0])*mc.D2R
        rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0])*mc.D2R
        rwPosVector = [[0.8, 0.8, 1.79070],
                       [0.8, -0.8, 1.79070],
                       [-0.8, -0.8, 1.79070],
                       [-0.8, 0.8, 1.79070]
                       ]

        gsHat = (rbk.Mi(-rwAzimuthAngle[0], 3).dot(rbk.Mi(rwElAngle[0], 2))).dot(np.array([1, 0, 0]))
        self.RW1 = self.rwFactory.create('Honeywell_HR16',
                                         gsHat,
                                         maxMomentum=maxRWMomentum,
                                         rWB_B=rwPosVector[0])
        
        gsHat = (rbk.Mi(-rwAzimuthAngle[1], 3).dot(rbk.Mi(rwElAngle[1], 2))).dot(np.array([1, 0, 0]))
        self.RW2 = self.rwFactory.create('Honeywell_HR16',
                                         gsHat,
                                         maxMomentum=maxRWMomentum,
                                         rWB_B=rwPosVector[1])

        gsHat = (rbk.Mi(-rwAzimuthAngle[2], 3).dot(rbk.Mi(rwElAngle[2], 2))).dot(np.array([1, 0, 0]))
        self.RW3 = self.rwFactory.create('Honeywell_HR16',
                                         gsHat,
                                         maxMomentum=maxRWMomentum,
                                         rWB_B=rwPosVector[2])
            
        gsHat = (rbk.Mi(-rwAzimuthAngle[3], 3).dot(rbk.Mi(rwElAngle[3], 2))).dot(np.array([1, 0, 0]))
        self.RW4 = self.rwFactory.create('Honeywell_HR16',
                                         gsHat,
                                         maxMomentum=maxRWMomentum,
                                         rWB_B=rwPosVector[3])

        self.rwFactory.addToSpacecraft("RWA", self.rwStateEffector, self.scObject)

    def SetACSThrusterStateEffector(self):
        """Set the 8 ACS thrusters."""
        # Make a fresh TH factory instance, this is critical to run multiple times
        thFactory = simIncludeThruster.thrusterFactory()

        # 8 thrusters are modeled that act in pairs to provide the desired torque
        thPos = [
            [825.5/1000.0, 880.3/1000.0, 1765.3/1000.0],
            [825.5/1000.0, 880.3/1000.0, 260.4/1000.0],
            [880.3/1000.0, 825.5/1000.0, 1765.3/1000.0],
            [880.3/1000.0, 825.5/1000.0, 260.4/1000.0],
            [-825.5/1000.0, -880.3/1000.0, 1765.3/1000.0],
            [-825.5/1000.0, -880.3/1000.0, 260.4/1000.0],
            [-880.3/1000.0, -825.5/1000.0, 1765.3/1000.0],
            [-880.3/1000.0, -825.5/1000.0, 260.4/1000.0]
                 ]
        thDir = [
            [0.0, -1.0, 0.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0]
                ]
        for pos_B, dir_B in zip(thPos, thDir):
            thFactory.create(
                'MOOG_Monarc_1'
                , pos_B
                , dir_B
            )
        # create thruster object container and tie to spacecraft object
        thFactory.addToSpacecraft("ACS Thrusters",
                                  self.thrustersDynamicEffectorACS,
                                  self.scObject)

    def SetDVThrusterStateEffector(self):
        """Set the DV thrusters."""
        # Make a fresh TH factory instance, this is critical to run multiple times
        thFactory = simIncludeThruster.thrusterFactory()

        # 6 DV thrusters
        thPos = [[0, 0.95, -1.1],
                 [0.8227241335952166, 0.4750000000000003, -1.1],
                 [0.8227241335952168, -0.47499999999999976, -1.1],
                 [0, -0.95, -1.1],
                 [-0.8227241335952165, -0.4750000000000004, -1.1],
                 [-0.822724133595217, 0.4749999999999993, -1.1]]
        thDir = [[0.0, 0.0, 1.0],
                 [0.0, 0.0, 1.0],
                 [0.0, 0.0, 1.0],
                 [0.0, 0.0, 1.0],
                 [0.0, 0.0, 1.0],
                 [0.0, 0.0, 1.0]]
        for pos_B, dir_B in zip(thPos, thDir):
            thFactory.create(
                'MOOG_Monarc_22_6'
                , pos_B
                , dir_B
            )
        # create thruster object container and tie to spacecraft object
        thFactory.addToSpacecraft("DV Thrusters",
                                  self.thrustersDynamicEffectorDV,
                                  self.scObject)


    def SetCSSConstellation(self):
        """Set the 8 CSS sensors"""
        self.CSSConstellationObject.ModelTag = "cssConstellation"

        def setupCSS(cssDevice):
            cssDevice.fov = 80. * mc.D2R         # half-angle field of view value
            cssDevice.scaleFactor = 2.0
            cssDevice.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
            cssDevice.stateInMsg.subscribeTo(self.scObject.scStateOutMsg)
            cssDevice.sunEclipseInMsg.subscribeTo(self.eclipseObject.eclipseOutMsgs[0])
            cssDevice.this.disown()

        # setup CSS sensor normal vectors in body frame components
        nHat_B_List = [
            [0.0, 0.707107, 0.707107],
            [0.707107, 0., 0.707107],
            [0.0, -0.707107, 0.707107],
            [-0.707107, 0., 0.707107],
            [0.0, -0.965926, -0.258819],
            [-0.707107, -0.353553, -0.612372],
            [0., 0.258819, -0.965926],
            [0.707107, -0.353553, -0.612372]
        ]
        numCSS = len(nHat_B_List)

        # store all
        cssList = []
        for nHat_B, i in zip(nHat_B_List, list(range(1,numCSS+1))):
            CSS = coarseSunSensor.CoarseSunSensor()
            setupCSS(CSS)
            CSS.ModelTag = "CSS" + str(i)
            CSS.nHat_B = np.array(nHat_B)
            cssList.append(CSS)

        # assign the list of CSS devices to the CSS array class
        self.CSSConstellationObject.sensorList = coarseSunSensor.CSSVector(cssList)

    # Method for adding reaction wheel faults
    def PeriodicRWFault(self, probability, faultType, fault, faultRW, currentTime):
        """
        Adds a fault periodically. Probability is the chance of the fault occurring per update.
        """
        if np.random.uniform() < probability:
            self.AddRWFault(faultType, fault, faultRW, currentTime)
        
        
    
    def AddRWFault(self, faultType, fault, faultRW, currentTime):
        """
        Adds a static friction fault to the reaction wheel.
        """
        self.RWFaultLog.append([faultType, fault, faultRW, currentTime*mc.NANO2MIN])
        if faultType == "friction":
            if faultRW == 1:
                self.RW1.fCoulomb += fault
            elif faultRW == 2:
                self.RW2.fCoulomb += fault
            elif faultRW == 3:
                self.RW3.fCoulomb += fault
            elif faultRW == 4:
                self.RW4.fCoulomb += fault
        else:
            print("Invalid fault type. No fault added.")

    # Global call to initialize every module
    def InitAllDynObjects(self):
        """
        Initialize all the dynamics objects.
        """
        self.SetSpacecraftHub()
        self.SetGravityBodies()
        self.SetExternalForceTorqueObject()
        self.SetSimpleNavObject()
        self.SetEclipseObject()
        self.SetCSSConstellation()
        #self.SetFuelTank()
        self.SetReactionWheelDynEffector()
        self.SetACSThrusterStateEffector()
        self.SetDVThrusterStateEffector()


