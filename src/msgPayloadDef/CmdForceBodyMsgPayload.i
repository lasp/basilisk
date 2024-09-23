%module(package="Basilisk.architecture.messaging") CmdForceBodyMsgPayload
%{
    #include "fswAlgorithms/fswUtilities/fswDefinitions.h"
    #include "simulation/dynamics/reactionWheels/reactionWheelSupport.h"
%}

%include "architecture/messaging/newMessaging.ih"

%include "msgPayloadDef/CmdForceBodyMsgPayload.h"
%{
    #include "msgPayloadDef/CmdForceBodyMsgPayload.h"
%}

INSTANTIATE_TEMPLATES(CmdForceBodyMsg)
