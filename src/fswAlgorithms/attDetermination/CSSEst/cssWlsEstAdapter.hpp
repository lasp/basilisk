//
// Created by Patrick Kenneally on 10/1/24.
//

#ifndef BASILISK_CSSWLAESTADAPTER_H
#define BASILISK_CSSWLAESTADAPTER_H

#include "fswAlgorithms/attDetermination/CSSEst/cssWlsEst.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/CSSConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/CSSUnitConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/CSSArraySensorMsgPayload.h"
#include "architecture/msgPayloadDefC/SunlineFilterMsgPayload.h"

#include "architecture/utilities/bskLogging.h"

class CssWlsEstAdapter : public SysModel {
public:
//    void Reset(uint64_t callTime) override;
//    void UpdateState(uint64_t callTime) override;

    ReadFunctor<CSSArraySensorMsgPayload> cssDataInMsg;                   //!< The name of the CSS sensor input message
    ReadFunctor<CSSConfigMsgPayload> cssConfigInMsg;                      //!< The name of the CSS configuration input message
    Message<NavAttMsgPayload> navStateOutMsg;                         //!< The name of the navigation output message containing the estimated states
    Message<SunlineFilterMsgPayload> cssWLSFiltResOutMsg;             //!< The name of the CSS filter data out message

    BSKLogger bskLogger={};                               //!< BSK Logging
    CssWlsEst cssWlsEstModule={};

    void readMessages() {
        this->cssWlsEstModule.cssDataInMsgPayload = this->cssDataInMsg();
    }

    void writeMessages(uint64_t currentSimNanos) {
        /*! - Output If the residual fit data if message linked*/
        if (this->cssWLSFiltResOutMsg.isLinked()) {
            this->cssWLSFiltResOutMsg.write(&this->cssWlsEstModule.cssWlsFiltResOutMsgPayload, this->moduleID, currentSimNanos);
        }
        /*! - If the status from the WLS computation good, populate the output messages with the computed data*/
        this->navStateOutMsg.write(&this->cssWlsEstModule.navStateOutMsgPayload, this->moduleID, currentSimNanos);
    }

    void Reset(uint64_t currentSimNanos) override {
        // check that required messages have been included
        if (!this->cssConfigInMsg.isLinked()) {
            this->bskLogger.bskLog(BSK_ERROR, "cssWlsEst.cssConfigInMsg wasn't connected.");
        }
        if (!this->cssDataInMsg.isLinked()) {
            this->bskLogger.bskLog(BSK_ERROR, "cssWlsEst.cssDataInMsg wasn't connected.");
        }

        this->cssWlsEstModule.cssConfigInMsgPayload = this->cssConfigInMsg();
        this->cssWlsEstModule.Reset(currentSimNanos);
    }

    void UpdateState(uint64_t currentSimNanos) override {
        this->readMessages();
        this->cssWlsEstModule.UpdateState(currentSimNanos);
        this->writeMessages(currentSimNanos);
    }
};

#endif // BASILISK_CSSWLAESTADAPTER_H
