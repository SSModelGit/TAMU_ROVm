within UWmBody;

package UWTypes "Constants and types with choices, especially to build menus"
  extends Modelica.Icons.TypesPackage;
  import Forces = UWmBody.UWForces;
  import Interfaces = UWmBody.UWInterfaces;
  import Frames = UWmBody.UWFrames;
  import Joints = UWmBody.UWJoints;
  import Parts = UWmBody.UWParts;
  import Sensors = UWmBody.UWSensors;
  import Visualizers = UWmBody.Visualizers;
  import Icons = UWmBody.UWIcons;
  annotation(Documentation(info = "<html>
<p>
In this package <b>types</b> and <b>constants</b> are defined that are used in the
MultiBody library. The types have additional annotation choices
definitions that define the menus to be built up in the graphical
user interface when the type is used as parameter in a declaration.
</p>
</html>"));
end UWTypes;
