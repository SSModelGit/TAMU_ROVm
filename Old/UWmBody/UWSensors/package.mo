within UWmBody;

package UWSensors "Sensors to measure variables"
  extends Modelica.Icons.SensorsPackage;
  import Forces = UWmBody.UWForces;
  import Interfaces = UWmBody.UWInterfaces;
  import Frames = UWmBody.UWFrames;
  import Joints = UWmBody.UWJoints;
  import Parts = UWmBody.UWParts;
  import Visualizers = UWmBody.Visualizers;
  import Types = UWmBody.UWTypes;
  import Icons = UWmBody.UWIcons;
  annotation(Documentation(info = "<html>
<p>
Package <b>Sensors</b> contains <b>ideal measurement</b>
components to determine absolute and relative kinematic
quantities, as well as cut-forces, cut-torques and power. All
measured quantities can be provided in every desired
coordinate system.
</p>
</html>"));
end UWSensors;
