%module(package="Basilisk.architecture.messaging") SunlineFilterMsgPayload
%{
    #include "fswAlgorithms/fswUtilities/fswDefinitions.h"
    #include "simulation/dynamics/reactionWheels/reactionWheelSupport.h"
%}

%include "architecture/messaging/newMessaging.ih"

// for RWAvailabilityMsgPayload
%include "fswAlgorithms/fswUtilities/fswDefinitions.h"
// for RWConfigMsgPayload
%include "simulation/dynamics/reactionWheels/reactionWheelSupport.h"

// for RWAvailabilityMsgPayload
ARRAYINTASLIST(FSWdeviceAvailability)
// for CSSConfigMsgPayload
STRUCTASLIST(CSSUnitConfigMsgPayload)
// for AccDataMsgPayload
STRUCTASLIST(AccPktDataMsgPayload)
// for RWConstellationMsgPayload
STRUCTASLIST(RWConfigElementMsgPayload)
// for CSSArraySensorMsgPayload
STRUCTASLIST(CSSArraySensorMsgPayload)
// for THRArrayConfigMsgPayload
STRUCTASLIST(THRConfigMsgPayload)
// for ReconfigBurnArrayInfoMsgPayload
STRUCTASLIST(ReconfigBurnInfoMsgPayload)
// for DataStorageStatusMsgPayload
%template(DoubleVector) std::vector<double, std::allocator<double>>;
// for DataStorageStatusMsgPayload
%template(StringVector) std::vector<std::string, std::allocator<std::string>>;

%include "msgPayloadDef/SunlineFilterMsgPayload.h"
%{
    #include "msgPayloadDef/SunlineFilterMsgPayload.h"
%}

INSTANTIATE_TEMPLATES(SunlineFilterMsg)
