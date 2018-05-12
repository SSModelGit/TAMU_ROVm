# Underwater ROV modeling at Texas A&M Unmanned Systems Labs
This is a library focused on developing a general underwater rigid body modeling framework in [Modelica](https://modelica.org), meant for modeling and simulating underwater ROV designs and controls. It is based off of the MultiBody library, and adds a water field component to allow for buoyancy and drag forces. It also contains basic connectivity to [ROS](http://www.ros.org), from the _[ModROS](https://github.com/SSModelGit/ModROS)_ library.

## Description of Packages
- __RBodyInFluid__ - library to be used for modeling underwater bodies - dependent on the MultiBody library, and MSL 3.2.2
- __ROVm__ - package for modeling the BlueROV2 underwater ROV - dependent on RBodyInFluid. 
- __modelica_ext_func__ - ROS package for controlling the ROSConnectedBlueROV2.mo example in ROVm. Based on _ModROS_.
