%module(package="Basilisk.architecture.messaging") CmdTorqueBodyMsgPayload
%include "architecture/messaging/newMessaging.ih"

%include "msgPayloadDef/CmdTorqueBodyMsgPayload.h"
%{
    #include "msgPayloadDef/CmdTorqueBodyMsgPayload.h"
%}

INSTANTIATE_TEMPLATES(CmdTorqueBodyMsg)
