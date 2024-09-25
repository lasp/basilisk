# README

### Basilisk

* [Summary of Basilisk](docs/source/index.rst)

### How do I get set up?
The following links contain installation instructions for the supported platforms:

- [Setup a macOS Development Environment](docs/source/Install/installOnMacOS.rst)

- [Setup a Linux Development Environment](docs/source/Install/installOnLinux.rst)

- [Setup a Windows Development Environment](docs/source/Install/installOnWindows.rst)



### Basilisk Development guidelines
* [Contributing](CONTRIBUTING.md)
* [Coding Guidelines](docs/source/Support/Developer/CodingGuidlines.rst)


### Getting Started
To get started with Basilisk, several tutorials are provided. The 
documentation lists the scenarios in an order that facilitates learning basic Basilisk features. In the source code they 
are stored under `src\examples\`. A good start would be to run `scenarioBasicOrbit.py`.

To run with the tutorials, it is suggested the user makes a copy of these tutorial files, and use the copies in order 
to learn, test and experiment. Copy the folder `{basiliskPath}/src/examples` into a new folder, and change to that 
directory.

To run the default scenario of `scenarioBasicOrbit`, in the directory of the copied tutorials, execute the python 
script: `python3 scenarioBasicOrbit.py`

Now, when you want to use a tutorial, navigate inside that folder, and edit and execute the *copied* integrated tests.

<!--Any new BSK module development should not occur within the BSK folder as this will be updated rapidly.  Rather, 
new FSW algorithm or simulation code modules should be created in a custom folder outside of the BSK directory.  A 
sample folder is provided named `BasiliskCustom` which contains sample FSW and Simulation modules.-->

We are working on the ability to develop custom BSK modules outside of the Basilisk folder.  This will enable 
developers to safely write their own BSK modules and still be able to do a drop in replacement of future BSK releases.

To use the standalone 3D Visualization, download the [Vizard](http://hanspeterschaub.info/basilisk/Vizard/Vizard.html).  
This is in development, but does provide a 3D view of many of the simulation states.  


### Who do I talk to?

Questions and answers are fielded in the project's [Github Discussions](https://github.com/lasp/basilisk/discussions).
