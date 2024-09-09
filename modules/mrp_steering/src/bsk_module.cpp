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
    struct foo {
        int x;
        float y;
    };

    struct demo final : public bsk::SysModel {
        struct {
            bsk::read_functor<float> rate;
            bsk::read_functor<foo> foo;
        } inputs;

        struct {
            bsk::message<foo> foo = {42, 101.0f};
            struct {
                bsk::message<double> volume;
            } bar;
        } outputs;

        void UpdateState(std::uint64_t CurrentSimNanos) override {
            *this->outputs.bar.volume += this->inputs.rate.getOrElse(4.2) * CurrentSimNanos;
            *this->outputs.foo = *this->inputs.foo;
        }

        bsk::inputs getInputs() override {
            return {
                {"rate", this->inputs.rate},
                {"foo", this->inputs.foo},
            };
        }

        bsk::outputs getOutputs() const override {
            return {
                {"foo", this->outputs.foo},
                {"bar", {
                    {"volume", this->outputs.bar.volume},
                }},
            };
        }
    };
}

template<>
struct bsk::schema<bsk::modules::foo> final {
    using carrier = bsk::modules::foo;

    static std::string repr() {
        return "bsk::modules::foo { \"x\": "
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

        void subscribe_to(plug const& source) {
            *this->payload = source.payload;
            *this->header = source.header;
        }

        void subscribe_to(bsk::plug const& other) override {
            auto source = dynamic_cast<plug const*>(&other);
            if (source == nullptr) throw bsk::mismatched_schemas_error();
            return this->subscribe_to(*source);
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

static_assert(bsk::is_schema<bsk::schema<bsk::modules::foo>>);

// mrp_steering basilisk adapter
namespace bsk::modules {
    struct mrp_steering final : public bsk::SysModel {
        struct {
            //! Current attitude error estimate (MRPs) of B relative to R
            bsk::read_functor<double[3]> sigma_BR;
        } inputs;

        struct {
            //! [r/s]   Desired body rate relative to R
            bsk::message<double[3]> omega_BastR_B;
            //! [r/s^2] Body-frame derivative of omega_BastR_B
            bsk::message<double[3]> omegap_BastR_B;
        } outputs;

        ::MrpSteering params;


        void UpdateState(std::uint64_t CurrentSimNanos [[maybe_unused]]) override {
            if (!this->inputs.sigma_BR.isLinked()) throw "TODO";

            MRPSteeringLaw(
                &this->params,
                *this->inputs.sigma_BR,
                *this->outputs.omega_BastR_B,
                *this->outputs.omegap_BastR_B );
        }

        bsk::inputs getInputs() override {
            return {
                {"sigma_BR", this->inputs.sigma_BR},
            };
        }

        bsk::outputs getOutputs() const override {
            return {
                {"omega_BastR_B", this->outputs.omega_BastR_B},
                {"omegap_BastR_B", this->outputs.omegap_BastR_B},
            };
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
