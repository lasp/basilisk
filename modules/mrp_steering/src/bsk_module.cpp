#include "../src/module.h"

#include "basilisk.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdio>
#include <cstdint>
#include <memory>
#include <string>

// custom schema demo
struct foo {
    int x;
    float y;
};

template<>
struct bsk::schema<foo> final {
    using carrier = foo;

    static std::string repr() {
        return "foo { \"x\": "
             + bsk::schema<int>::repr()
             + ", \"y\": "
             + bsk::schema<float>::repr()
             + " }";
    }

    struct __attribute__((visibility("default"))) plug : public bsk::plug {
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

        std::string repr() const override {
            return schema::repr();
        }
    };
};

static_assert(bsk::is_schema<bsk::schema<foo>>);

struct BskFooModule final : public bsk::SysModel {
private:
    bsk::read_functor<float> rateInMsg;
    bsk::read_functor<foo> fooInMsg;

    bsk::message<double> volumeOutMsg = 0.0;
    bsk::message<foo> fooOutMsg = {0, 0.0f};

public:
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
};

// mrp_steering basilisk adapter
class __attribute__((visibility("default"))) BskMrpSteering final : public bsk::SysModel {
private:
    //! Current attitude error estimate (MRPs) of B relative to R
    bsk::read_functor<double[3]> sigma_BR;
    //! [r/s]   Desired body rate relative to R
    bsk::message<double[3]> omega_BastR_B = {0.0};
    //! [r/s^2] Body-frame derivative of omega_BastR_B
    bsk::message<double[3]> omegap_BastR_B = {0.0};

public:
    MrpSteering params;

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


// common basilisk pybind glue
namespace py = pybind11;

/*! @brief A trampoline for implementations of SysModel within Python */
class PySysModel final : public bsk::SysModel {
public:
    /* Inherit the constructors */
    using bsk::SysModel::SysModel;

    void SelfInit() override {
        PYBIND11_OVERRIDE(void, bsk::SysModel, SelfInit, /*no parameters*/);
    }
    void IntegratedInit() override {
        PYBIND11_OVERRIDE(void, bsk::SysModel, IntegratedInit, /*no parameters*/);
    }
    void UpdateState(uint64_t CurrentSimNanos) override {
        PYBIND11_OVERRIDE(void, bsk::SysModel, UpdateState, CurrentSimNanos);
    }
    void Reset(uint64_t CurrentSimNanos) override {
        PYBIND11_OVERRIDE(void, bsk::SysModel, Reset, CurrentSimNanos);
    }
    bsk::inputs getInputs() override {
        // TODO: actually, how should `getInputs` work for Python modules?
        // we need some way for a python module to allocate typed primitive
        // pointers that can reference data from C++ modules
        PYBIND11_OVERRIDE(bsk::inputs, bsk::SysModel, getInputs, /*no parameters*/);
    }
    bsk::outputs getOutputs() const override {
        // TODO: actually, how should `getOutputs` work for Python modules?
        // we need some way for a python module to allocate typed primitives
        // that are referencable from C++ modules
        PYBIND11_OVERRIDE(bsk::outputs, bsk::SysModel, getOutputs, /*no parameters*/);
    }
};

void subscribeToPy(bsk::socket& target, py::object source) {
    try {
        // if the source is a regular plug, drop into the core subscription logic.
        return target.subscribeTo(*source.cast<std::shared_ptr<bsk::plug>>());
    } catch (py::cast_error const& ex) {
        // otherwise, it's assumed to be a further structured Python object.
        // that means we need entry.second to be a bsk::inputs.
        auto components = dynamic_cast<bsk::inputs*>(&target);
        if (components == nullptr) throw bsk::mismatched_schemas_error();

        for (auto entry : *components) {
            subscribeToPy(*entry.second, source[py::cast(entry.first)]);
        }
    }
}

PYBIND11_MODULE(mrp_steering, m) {
    // Basilisk types (more common basilisk pybind glue)
    {
        py::class_<bsk::plug, std::shared_ptr<bsk::plug>>(m, "Plug")
            .def("__repr__", &bsk::plug::repr)
            .def("__getitem__", py::overload_cast<std::string>(&bsk::plug::focus, py::const_))
            .def("__getitem__", py::overload_cast<std::size_t>(&bsk::plug::focus, py::const_))
            .def("__getitem__", [](bsk::plug const& self, py::slice range) {
                self.focus(range.attr("start").cast<size_t>(), range.attr("stop").cast<size_t>());
            })
            .def_property_readonly("indices", &bsk::plug::focusable_indices)
            .def_property_readonly("names", &bsk::plug::focusable_names);

        py::class_<bsk::socket, std::shared_ptr<bsk::socket>>(m, "Socket")
            .def("__repr__", &bsk::socket::repr)
            .def("__getitem__", py::overload_cast<std::string>(&bsk::socket::focus))
            .def("__getitem__", py::overload_cast<std::size_t>(&bsk::socket::focus))
            .def("__getitem__", [](bsk::socket& self, py::slice range) {
                self.focus(range.attr("start").cast<size_t>(), range.attr("stop").cast<size_t>());
            })
            .def_property_readonly("indices", &bsk::socket::focusable_indices)
            .def_property_readonly("names", &bsk::socket::focusable_names)
            .def("subscribeTo", &subscribeToPy);

        py::class_<bsk::SysModel, std::shared_ptr<bsk::SysModel>, PySysModel>(m, "SysModel")
            .def(py::init<>())
            .def("SelfInit", &bsk::SysModel::SelfInit)
            .def("IntegratedInit", &bsk::SysModel::IntegratedInit)
            .def("UpdateState", &bsk::SysModel::UpdateState)
            .def("Reset", &bsk::SysModel::Reset)
            .def_readwrite("ModelTag", &bsk::SysModel::ModelTag)
            .def_readwrite("CallCounts", &bsk::SysModel::CallCounts)
            .def_readwrite("RNGSeed", &bsk::SysModel::RNGSeed)
            .def_readwrite("moduleID", &bsk::SysModel::moduleID)
            .def("subscribeTo", [](bsk::SysModel& self, py::object source) {
                auto inputs = self.getInputs();
                subscribeToPy(inputs, source);
            })
            .def_property_readonly("inputs", [](bsk::SysModel& self) -> std::shared_ptr<bsk::socket> {
                return std::make_shared<bsk::inputs>(self.getInputs());
            })
            .def_property_readonly("outputs", [](bsk::SysModel& self) -> std::shared_ptr<bsk::plug> {
                return std::make_shared<bsk::outputs>(self.getOutputs());
            });
    }

    // more custom schema demo
    {
        py::class_<BskFooModule, bsk::SysModel, std::shared_ptr<BskFooModule>>(m, "FooModule", py::module_local())
            .def(py::init<>());
    }

    // more mrp_steering basilisk adapter
    {
        auto module = py::class_<BskMrpSteering, std::shared_ptr<BskMrpSteering>, bsk::SysModel>(m, "MrpSteering", py::is_final(), py::module_local())
            .def(py::init<>())
            .def_readwrite("parameters", &BskMrpSteering::params);

        py::class_<MrpSteering>(module, "Parameters", py::is_final(), py::module_local())
            .def(py::init<>())
            .def_readwrite("K1", &MrpSteering::K1)
            .def_readwrite("K3", &MrpSteering::K3)
            .def_readwrite("omega_max", &MrpSteering::omega_max)
            .def_readwrite("ignoreOuterLoopFeedforward", &MrpSteering::ignoreOuterLoopFeedforward);

        m.def("MRPSteeringLaw", &MRPSteeringLaw);
    }
}
