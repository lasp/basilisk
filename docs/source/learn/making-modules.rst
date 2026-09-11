.. _making-modules:

Xmera Modules
================

This chapter covers the fundamentals of designing Xmera modules and how to write a module
using C++, C, or Python. As with all code documentation, it is never truly complete and is
always a work in progress.

For general developer resources—including coding guidelines, forking the repository,
building the Sphinx documentation system, and more—see the ``Support/Developer Information``
section in :ref:`making-modules`.

The following pages walk through the primary tasks required to create a Xmera module.
Topics include how to design a module, how the message-passing system works, what methods
are required in a module, and more. Note that dynamics modules are both Xmera modules
and subclasses of either:

* :ref:`dynamicEffector` — provides external forces and torques acting on the spacecraft,
  but does not have internal state differential equations.
* :ref:`stateEffector` — includes internal state differential equations that couple
  with the spacecraft rigid body.

.. warning::

    The goal of this documentation is to support the module creation process. However,
    it does **not** eliminate the need to study the Xmera source code, particularly
    how existing modules are implemented, to fully understand the module architecture.

.. toctree::
   :maxdepth: 2

   making-modules/modules
   making-modules/swig-module-interface
   making-modules/module-design
   making-modules/messaging/overview
   making-modules/messaging/message-objects
   making-modules/messaging/creating-message-types
   making-modules/messaging/mission-parameters
   making-modules/module-adapter
   making-modules/module-python
   making-modules/module-testing
   making-modules/module-documentation
   making-modules/module-utilities
   making-modules/cppModules/module-header
   making-modules/cppModules/module-definition

   making-modules/advancedTopics
