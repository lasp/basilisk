/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

*/

%module vizInterface
%{
    #include "vizInterface.h"
    #include "simulation/vizard/_GeneralModuleFiles/vizStructures.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_conly_data.i"
%include "swig_eigen.i"

%include "sys_model.i"
%include "std_vector.i"

// Instantiate templates used by example
namespace std {
    %template(PointLineConfig) vector<PointLine>;
    %template(LocationConfig) vector<LocationPbMsg *>;
    %template(CustomModelConfig) vector<CustomModel>;
    %template(ActuatorGuiSettingsConfig) vector<ActuatorGuiSettings>;
    %template(InstrumentGuiSettingsConfig) vector<InstrumentGuiSettings>;
    %template(KeepOutInConeConfig) vector<KeepOutInCone>;
    %template(StdCameraConfig) vector<StdCameraSettings>;
    %template(VizSCVector) vector<VizSpacecraftData>;
    %template(ThrClusterVector) vector<ThrClusterMap>;
    %template(GravBodyInfoVector) vector<GravBodyInfo>;
    %template(GenericSensorVector) vector<GenericSensor *>;
    %template(LightVector) vector<Light *>;
    %template(TransceiverVector) vector<Transceiver *>;
    %template(GenericStorageVector) vector<GenericStorage *>;
    %template(MultiSphereVector) vector<MultiSphere *>;
    %template(EllipsoidVector) vector<Ellipsoid *>;
}

%include "vizInterface.h"
%include "simulation/vizard/_GeneralModuleFiles/vizStructures.h"

%include "msgPayloadDef/CameraConfigMsgPayload.h"
%include "msgPayloadDef/RWConfigLogMsgPayload.h"
%include "msgPayloadDef/SCStatesMsgPayload.h"
%include "msgPayloadDef/CameraImageMsgPayload.h"
%include "msgPayloadDef/SpicePlanetStateMsgPayload.h"
%include "msgPayloadDef/RWSpeedMsgPayload.h"
%include "msgPayloadDef/EpochMsgPayload.h"
%include "msgPayloadDef/CSSConfigLogMsgPayload.h"
%include "msgPayloadDef/THROutputMsgPayload.h"
%include "msgPayloadDef/ChargeMsmMsgPayload.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
