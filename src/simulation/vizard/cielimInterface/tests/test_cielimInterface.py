# ISC License
#
# Copyright (c) 2024, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import tempfile, sys
from Basilisk import __path__
bskPath = __path__[0]
sys.path.append(bskPath + "/../../src/utilities/vizProtobuffer/")
import cielimMessage_pb2
import delimited_protobuf
import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.utilities import SimulationBaseClass

importErr = False
reasonErr = ""
try:
    from Basilisk.simulation import cielimInterface
except ImportError:
    importErr = True
    reasonErr = "\nCielim Interface not built ---check build option"

@pytest.mark.skipif(importErr, reason=reasonErr)
def test_interface():
    read_write_test()

def read_write_test():
    tmpdir = tempfile.TemporaryDirectory()
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = int(1E9)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = cielimInterface.CielimInterface()
    module.ModelTag = "cielim_interface"
    module.setOpNavMode(cielimInterface.ClosedLoopMode_OPEN_LOOP)
    module.setSaveFile(tmpdir.name + "/test_proto")

    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Create epoch message
    epoch_payload = messaging.EpochMsgPayload()
    epoch_payload.year = 2050
    epoch_payload.month = 2
    epoch_payload.day = 15
    epoch_payload.hours = 22
    epoch_payload.minutes = 8
    epoch_payload.seconds = 58
    epoch_message = messaging.EpochMsg().write(epoch_payload)
    module.epochMessage.subscribeTo(epoch_message)

    # Create camera rendering message
    rendering_payload = messaging.CameraRenderingMsgPayload()
    rendering_payload.cameraId = 1
    rendering_payload.cosmicRayStdDeviation = 0.1
    rendering_payload.enableStrayLight = True
    rendering_payload.starField = True
    rendering_payload.rendering = "Lumen"
    rendering_payload.smear = True
    rendering_message = messaging.CameraRenderingMsg().write(rendering_payload)
    module.cameraRenderingMessage.subscribeTo(rendering_message)

    # Create asteroid parameter message
    asteroid_parameter_payload = messaging.CelestialBodyParametersMsgPayload()
    asteroid_parameter_payload.bodyName = "asteroid_b612"
    asteroid_parameter_payload.shapeModel = "Bennu"
    asteroid_parameter_payload.perlinNoise = 0.5
    asteroid_parameter_payload.proceduralRocks = 1.5
    asteroid_parameter_payload.brdf = "Lambertian"
    asteroid_parameter_payload.reflectanceParameters = [0.5]
    asteroid_parameter_payload.meanRadius = 50000
    asteroid_parameter_payload.principalAxisDistortion = [10, 5, 10]
    asteroid_parameter_message = messaging.CelestialBodyParametersMsg().write(asteroid_parameter_payload)
    module.celestialParametersMessage.subscribeTo(asteroid_parameter_message)

    # Create camera message
    camera_payload = messaging.CameraModelMsgPayload()
    camera_payload.timeTag = int(0.5*1E9)
    camera_payload.cameraId = 1
    camera_payload.parentName = "cielim_sat"
    camera_payload.fieldOfView = [20*np.pi/180, 15*np.pi/180]
    camera_payload.bodyToCameraMrp = [1/3,-1/3,-1/3]
    camera_payload.cameraBodyFramePosition = [1,1,1]
    camera_payload.resolution = [4000, 3000]
    camera_payload.renderRate = 10000
    camera_payload.focalLength = 0.1
    camera_payload.readNoise = 1
    camera_payload.systemGain = 0.5
    camera_payload.gaussianPointSpreadFunction = 2
    camera_payload.exposureTime = 0.1
    camera_message = messaging.CameraModelMsg().write(camera_payload)
    module.cameraModelMessage.subscribeTo(camera_message)

    # Create spacecraft message
    spacecraft_payload = messaging.SCStatesMsgPayload()
    spacecraft_payload.r_BN_N = [0,0,-5E4]
    spacecraft_payload.v_BN_N = [4, 5, 6]
    spacecraft_payload.sigma_BN = [np.sqrt(2)/2, 0, -np.sqrt(2)/2]
    spacecraft_message = messaging.SCStatesMsg().write(spacecraft_payload)
    module.spacecraftMessage.subscribeTo(spacecraft_message)

    # Create spice body messages using a similar structure to the factory (for ease of implementation)
    class GravBodies:
        planetName = ""
        isCentralBody = False
        planetBodyInMsg = messaging.SpicePlanetStateMsg()

    grav_bodies = []
    bodies_message_list = []
    sun = GravBodies()
    sun.planetName = "sun_planet_data"
    sun_data = messaging.SpicePlanetStateMsgPayload()
    sun_data.PositionVector = [10E10,0,0]
    sun_data.VelocityVector = [4, 5, 6]
    sun_data.J20002Pfix = np.eye(3)
    sun.planetBodyInMsg = messaging.SpicePlanetStateMsg().write(sun_data)
    grav_bodies.append(sun)
    bodies_message_list.append(sun_data)

    asteroid_b612 = GravBodies()
    asteroid_b612.planetName = "asteroid_b612"
    asteroid_b612.isCentralBody = True
    asteroid_data = messaging.SpicePlanetStateMsgPayload()
    asteroid_data.PositionVector = [0,0,0]
    asteroid_data.VelocityVector = [4, 5, 6]
    asteroid_data.J20002Pfix = -np.eye(3)
    asteroid_b612.planetBodyInMsg = messaging.SpicePlanetStateMsg().write(asteroid_data)
    grav_bodies.append(asteroid_b612)
    bodies_message_list.append(asteroid_data)

    for gravBody in grav_bodies:
        body = cielimInterface.SpiceBody()
        body.name = gravBody.planetName
        body.isCentralBody = gravBody.isCentralBody
        body.spiceStateMessage.subscribeTo(gravBody.planetBodyInMsg)
        module.addCelestialBody(body)

    # Run block
    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(int(1e9))
    unit_test_sim.ExecuteSimulation()
    module.closeProtobufFile()

    # Read the existing address book.
    file_handle = open(module.getSaveFilename(), "rb")
    cielim_message = delimited_protobuf.read(file_handle, cielimMessage_pb2.CielimMessage)

    np.testing.assert_equal(cielim_message.currentTime.frameNumber, 1)
    np.testing.assert_equal(cielim_message.currentTime.simTimeElapsed, test_process_rate)

    np.testing.assert_equal(cielim_message.epoch.year, epoch_payload.year)
    np.testing.assert_equal(cielim_message.epoch.month, epoch_payload.month)
    np.testing.assert_equal(cielim_message.epoch.day, epoch_payload.day)
    np.testing.assert_equal(cielim_message.epoch.hours, epoch_payload.hours)
    np.testing.assert_equal(cielim_message.epoch.minutes, epoch_payload.minutes)
    np.testing.assert_equal(cielim_message.epoch.seconds, epoch_payload.seconds)

    np.testing.assert_equal(cielim_message.spacecraft.spacecraftName, "cielim_sat")
    np.testing.assert_equal(cielim_message.spacecraft.position, spacecraft_payload.r_BN_N)
    np.testing.assert_equal(cielim_message.spacecraft.velocity, spacecraft_payload.v_BN_N)
    np.testing.assert_equal(cielim_message.spacecraft.attitude, spacecraft_payload.sigma_BN)

    np.testing.assert_equal(cielim_message.camera.cameraId, camera_payload.cameraId)
    np.testing.assert_equal(cielim_message.camera.parentName, "cielim_sat")
    np.testing.assert_equal(cielim_message.camera.fieldOfView,camera_payload.fieldOfView)
    np.testing.assert_equal(cielim_message.camera.resolution, camera_payload.resolution)
    np.testing.assert_equal(cielim_message.camera.cameraPositionInBody, camera_payload.cameraBodyFramePosition)
    np.testing.assert_equal(cielim_message.camera.bodyFrameToCameraMrp, camera_payload.bodyToCameraMrp)
    np.testing.assert_equal(cielim_message.camera.renderRate, camera_payload.renderRate)
    np.testing.assert_equal(cielim_message.camera.focalLength, camera_payload.focalLength)
    np.testing.assert_equal(cielim_message.camera.exposureTime, camera_payload.exposureTime)
    np.testing.assert_equal(cielim_message.camera.pointSpreadFunction, camera_payload.gaussianPointSpreadFunction)
    np.testing.assert_equal(cielim_message.camera.systemGain, camera_payload.systemGain)
    np.testing.assert_equal(cielim_message.camera.readNoise, camera_payload.readNoise)

    np.testing.assert_equal(cielim_message.camera.renderParameters.cosmicRayStdDeviation, rendering_payload.cosmicRayStdDeviation)
    np.testing.assert_equal(cielim_message.camera.renderParameters.enableStrayLight, rendering_payload.enableStrayLight)
    np.testing.assert_equal(cielim_message.camera.renderParameters.starField, rendering_payload.starField)
    np.testing.assert_equal(cielim_message.camera.renderParameters.rendering, rendering_payload.rendering)
    np.testing.assert_equal(cielim_message.camera.renderParameters.enableSmear, rendering_payload.smear)

    i = 0
    for message, body in zip(bodies_message_list, grav_bodies):
        name = body.planetName
        central = body.isCentralBody
        np.testing.assert_equal(cielim_message.celestialBodies[i].bodyName, name)
        np.testing.assert_equal(cielim_message.celestialBodies[i].position, message.PositionVector)
        np.testing.assert_equal(cielim_message.celestialBodies[i].velocity, message.VelocityVector)
        np.testing.assert_equal(cielim_message.celestialBodies[i].attitude, np.array(message.J20002Pfix).flatten())
        np.testing.assert_equal(cielim_message.celestialBodies[i].centralBody, central)
        if (name == asteroid_parameter_payload.bodyName):
            np.testing.assert_equal(cielim_message.celestialBodies[i].models.shapeModel, asteroid_parameter_payload.shapeModel)
            np.testing.assert_equal(cielim_message.celestialBodies[i].models.perlinNoiseStdDeviation, asteroid_parameter_payload.perlinNoise)
            np.testing.assert_equal(cielim_message.celestialBodies[i].models.proceduralRocks, asteroid_parameter_payload.proceduralRocks)
            np.testing.assert_equal(cielim_message.celestialBodies[i].models.brdfModel, asteroid_parameter_payload.brdf)
            np.testing.assert_equal(cielim_message.celestialBodies[i].models.reflectanceParameters, asteroid_parameter_payload.reflectanceParameters)
            np.testing.assert_equal(cielim_message.celestialBodies[i].models.meanRadius, asteroid_parameter_payload.meanRadius)
            np.testing.assert_equal(cielim_message.celestialBodies[i].models.principalAxisDistortion, asteroid_parameter_payload.principalAxisDistortion)
        i += 1

if __name__ == "__main__":
    test_interface()
