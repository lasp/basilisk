#include "module.h"
#include "basilisk.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdio>
#include <cstdint>
#include <memory>
#include <string>

// Python support for binding Basilisk modules into Python.
namespace bsk::py {
    template<typename T>
    auto module_(pybind11::module_& m, char const name[]) {
        return pybind11::class_<T, bsk::SysModel, std::shared_ptr<T>>(m, name, pybind11::is_final(), pybind11::module_local());
    }
}

// custom schema demo
namespace bsk::modules {
    struct demo final : public bsk::SysModel {
        struct foo {
            int x;
            float y;
        };

        bsk::inputs getInputs() override {
            return {
                {"rate", this->rateInMsg},
                {"foo", this->fooInMsg},
            };
        }

        bsk::outputs getOutputs() const override {
            return {
                {"foo", this->fooOutMsg},
                {"bar", {
                    {"baz", this->volumeOutMsg},
                }},
            };
        }

        void UpdateState(std::uint64_t CurrentSimNanos) override {
            *this->volumeOutMsg += this->rateInMsg.getOrElse(4.2) * CurrentSimNanos;
            *this->fooOutMsg = *this->fooInMsg;
        }

    private:
        bsk::read_functor<float> rateInMsg;
        bsk::read_functor<foo> fooInMsg;

        bsk::message<double> volumeOutMsg = 0.0;
        bsk::message<foo> fooOutMsg = {42, 101.0f};
    };
}

template<>
struct bsk::schema<bsk::modules::demo::foo> final {
    using carrier = bsk::modules::demo::foo;

    static std::string repr() {
        return "bsk::modules::demo::foo { \"x\": "
             + bsk::schema<int>::repr()
             + ", \"y\": "
             + bsk::schema<float>::repr()
             + " }";
    }

    struct plug : public bsk::plug {
        bsk::message_header const* const header;
        carrier const* const payload;

        plug(bsk::message_header const* const header, carrier const* const payload)
            : header(header)
            , payload(payload)
        {}

        bsk::plug_for<int> pick_x() const {
            return bsk::plug_for<int>(this->header, &this->payload->x);
        }

        bsk::plug_for<float> pick_y() const {
            return bsk::plug_for<float>(this->header, &this->payload->y);
        }

        std::vector<std::string> focusable_names() const override {
            return {"x", "y"};
        }

        std::shared_ptr<bsk::plug> focus(std::string const name) const override {
            if (name == "x") return std::make_shared<bsk::plug_for<int>>(this->pick_x());
            if (name == "y") return std::make_shared<bsk::plug_for<float>>(this->pick_y());
            throw bsk::illegal_focus_error();
        }

        std::string repr() const override {
            return schema::repr();
        }
    };

    struct socket : public bsk::socket {
        bsk::message_header const** const header;
        carrier const** const payload;

        socket(bsk::message_header const** const header, carrier const** const payload)
            : header(header)
            , payload(payload)
        {}

        void subscribeTo(plug const& source) {
            *this->payload = source.payload;
            *this->header = source.header;
        }

        void subscribeTo(bsk::plug const& other) override {
            auto source = dynamic_cast<plug const*>(&other);
            if (source == nullptr) throw bsk::mismatched_schemas_error();
            return this->subscribeTo(*source);
        }

        bool can_subscribe_to(bsk::plug const& other) const override {
            auto source = dynamic_cast<plug const*>(&other);
            if (source == nullptr) return false;
            return true;
        }

        std::string repr() const override {
            return schema::repr();
        }
    };
};

static_assert(bsk::is_schema<bsk::schema<bsk::modules::demo::foo>>);

// mrp_steering basilisk adapter
namespace bsk::modules {
    class mrp_steering final : public bsk::SysModel {
    private:
        //! Current attitude error estimate (MRPs) of B relative to R
        bsk::read_functor<double[3]> sigma_BR;
        //! [r/s]   Desired body rate relative to R
        bsk::message<double[3]> omega_BastR_B = {0.0};
        //! [r/s^2] Body-frame derivative of omega_BastR_B
        bsk::message<double[3]> omegap_BastR_B = {0.0};

    public:
        ::MrpSteering params;

        bsk::inputs getInputs() override {
            return {
                {"sigma_BR", this->sigma_BR},
            };
        }

        bsk::outputs getOutputs() const override {
            return {
                {"omega_BastR_B", this->omega_BastR_B},
                {"omegap_BastR_B", this->omegap_BastR_B},
            };
        }

        void UpdateState(std::uint64_t CurrentSimNanos) override {
            if (!this->sigma_BR.isLinked()) throw "TODO";

            MRPSteeringLaw(
                &this->params,
                *this->sigma_BR,
                *this->omega_BastR_B,
                *this->omegap_BastR_B );
        }
    };
}


namespace py = pybind11;

PYBIND11_MODULE(mrp_steering, m) {
    m.import("basilisk");

    bsk::py::module_<bsk::modules::demo>(m, "DemoModule")
        .def(py::init<>());

    auto module = bsk::py::module_<bsk::modules::mrp_steering>(m, "MrpSteeringModule")
        .def(py::init<>())
        .def_readwrite("parameters", &bsk::modules::mrp_steering::params);

    py::class_<MrpSteering>(module, "Parameters", py::is_final(), py::module_local())
        .def(py::init<>())
        .def_readwrite("K1", &MrpSteering::K1)
        .def_readwrite("K3", &MrpSteering::K3)
        .def_readwrite("omega_max", &MrpSteering::omega_max)
        .def_readwrite("ignoreOuterLoopFeedforward", &MrpSteering::ignoreOuterLoopFeedforward);

    m.def("MRPSteeringLaw", &MRPSteeringLaw);
}
