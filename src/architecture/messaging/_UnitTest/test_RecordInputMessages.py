#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

from numpy import testing

from Basilisk.architecture import messaging
from Basilisk.moduleTemplates import cppModuleTemplate
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


def test_RecordingInputMessages():
    """
    testing recording an input message
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(1.)))

    # create modules
    mod1 = cppModuleTemplate.CppModuleTemplate()
    mod1.ModelTag = "module1"
    scSim.AddModelToTask("dynamicsTask", mod1)

    # Write input data
    inputData = messaging.CModuleTemplateMsgPayload()
    inputData.dataVector = [1, 2, 3]
    inputDataMsg = messaging.CModuleTemplateMsg().write(inputData)

    # Subscribe input message to stand-alone message
    mod1.dataInMsg.subscribeTo(inputDataMsg)

    attGuidMsg = messaging.CModuleTemplateMsg()
    attGuidMsgPayload = messaging.CModuleTemplateMsgPayload()
    attGuidMsg.write(attGuidMsgPayload)
    mod1.dataOutMsg = attGuidMsg

    # Create recorders tied to IO messages
    dataInRec = mod1.dataInMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", dataInRec)
    dataOutRec = mod1.dataOutMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", dataOutRec)
    dataOut2Rec = attGuidMsg.recorder()
    scSim.AddModelToTask("dynamicsTask", dataOut2Rec)

    #  initialize Simulation:
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(1.0))
    scSim.ExecuteSimulation()

    # reading the module output message show not change the earlier redirection
    # further, we are testing that the read() command copies the payload from
    # the stand-alone msg to the module output module
    tempSet = mod1.dataOutMsg.read().dataVector
    scSim.ConfigureStopTime(macros.sec2nano(2.0))
    scSim.ExecuteSimulation()

    testing.assert_allclose([inputData.dataVector]*3,
                            dataInRec.dataVector,
                            atol=0.01,
                            err_msg="recorded input message was not correct.")

    testing.assert_allclose([[2, 2, 3], [3, 2, 3], [4, 2, 3]],
                            dataOutRec.dataVector,
                            atol=0.01,
                            err_msg="recorded module output message was not correct.")

    testing.assert_equal(dataOutRec.dataVector,
                         dataOut2Rec.dataVector,
                         err_msg="redirected recording does not match original recording.")

    testing.assert_equal(mod1.dataOutMsg.read().dataVector,
                         attGuidMsg.read().dataVector,
                         err_msg="read of module output message was not correct.")


if __name__ == "__main__":
    test_RecordingInputMessages()
