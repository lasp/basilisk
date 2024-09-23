%module(package="Basilisk.architecture.messaging") THRArrayCmdForceMsgPayload

%include "architecture/messaging/newMessaging.ih"

%include "msgPayloadDef/THRArrayCmdForceMsgPayload.h"
%{
    #include "msgPayloadDef/THRArrayCmdForceMsgPayload.h"
%}

INSTANTIATE_TEMPLATES(THRArrayCmdForceMsg)
