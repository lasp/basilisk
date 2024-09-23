%module(package="Basilisk.architecture.messaging") VehicleConfigMsgPayload
%include "architecture/messaging/newMessaging.ih"

%include "msgPayloadDef/VehicleConfigMsgPayload.h"
%{
    #include "msgPayloadDef/VehicleConfigMsgPayload.h"
%}

INSTANTIATE_TEMPLATES(VehicleConfigMsg)
