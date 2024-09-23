%module(package="Basilisk.architecture.messaging") THRArrayConfigMsgPayload
%include "architecture/messaging/newMessaging.ih"

// for THRArrayConfigMsgPayload
STRUCTASLIST(THRConfigMsgPayload)

%include "msgPayloadDef/THRArrayConfigMsgPayload.h"
%{
    #include "msgPayloadDef/THRArrayConfigMsgPayload.h"
%}

INSTANTIATE_TEMPLATES(THRArrayConfigMsg)
