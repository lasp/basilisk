#include "basilisk.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdio>

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

static void subscribe_to_py(bsk::socket& target, py::object source) {
    try {
        // if the source is a regular plug, drop into the core subscription logic.
        return target.subscribe_to(*source.cast<std::shared_ptr<bsk::plug>>());
    } catch (py::cast_error const& ex) {
        // otherwise, it's assumed to be a further structured Python object.
        // that means we need entry.second to be a bsk::inputs.
        auto components = dynamic_cast<bsk::inputs*>(&target);
        if (components == nullptr) throw bsk::mismatched_schemas_error();

        for (auto entry : *components) {
            subscribe_to_py(*entry.second, source[py::cast(entry.first)]);
        }
    }
}

static bool can_subscribe_to_py(bsk::socket& target, py::object source) {
    try {
        // if the source is a regular plug, drop into the core subscription logic.
        return target.can_subscribe_to(*source.cast<std::shared_ptr<bsk::plug>>());
    } catch (py::cast_error const& ex) {
        // otherwise, it's assumed to be a further structured Python object.
        // that means we need entry.second to be a bsk::inputs.
        auto components = dynamic_cast<bsk::inputs*>(&target);
        if (components == nullptr) return false;

        for (auto entry : *components) {
            if (!can_subscribe_to_py(*entry.second, source[py::cast(entry.first)])) {
                return false;
            }
        }

        return true;
    }
}

PYBIND11_MODULE(core, m) {
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
        .def("canSubscribeTo", &can_subscribe_to_py)
        .def("subscribeTo", &subscribe_to_py);

    py::class_<bsk::fanout, bsk::socket, std::shared_ptr<bsk::fanout>>(m, "Fanout")
        .def(py::init<std::vector<std::shared_ptr<bsk::socket>>>());

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
            subscribe_to_py(inputs, source);
        })
        .def_property_readonly("inputs", [](bsk::SysModel& self) -> std::shared_ptr<bsk::socket> {
            return std::make_shared<bsk::inputs>(self.getInputs());
        })
        .def_property_readonly("outputs", [](bsk::SysModel& self) -> std::shared_ptr<bsk::plug> {
            return std::make_shared<bsk::outputs>(self.getOutputs());
        });
}
