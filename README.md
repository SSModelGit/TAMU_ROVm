# Underwater ROV modeling at Texas A&M Unmanned Systems Labs
This is a library focused on developing a general underwater rigid body modeling framework in [Modelica](https://modelica.org), meant for modeling and simulating underwater ROV designs and controls. It is based off of the MultiBody library, and adds a water field component to allow for buoyancy and drag forces. It also contains basic connectivity to [ROS](http://www.ros.org), from the _[modelica_bridge](https://github.com/ModROS/modelica_bridge)_ library (previously known as _ModROS_).

## Description of Packages
- __URBL__ - The Underwater Rigid Body Library is to be used for modeling underwater bodies - dependent on the MultiBody library, and MSL 3.2.2
- __ROVm__ - package for modeling the BlueROV2 underwater ROV - dependent on RBodyInFluid. 
- The ROS package for connecting the Modelica models to ROS has now been extended for general use, and packaged into [__modelica_bridge__](https://github.com/ModROS/modelica_bridge)
