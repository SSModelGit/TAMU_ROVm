within UWmBody;

package UWInterfaces "Connectors and partial models for 3-dim. mechanical components"
  extends Modelica.Icons.InterfacesPackage;
  import Forces = UWmBody.UWForces;
  import Frames = UWmBody.UWFrames;
  import Joints = UWmBody.UWJoints;
  import Parts = UWmBody.UWParts;
  import Sensors = UWmBody.UWSensors;
  import Visualizers = UWmBody.Visualizers;
  import Types = UWmBody.UWTypes;
  import Icons = UWmBody.UWIcons;
  annotation(Documentation(info = "<html>
<p>
This package contains connectors and partial models (i.e., models
that are only used to build other models) of the MultiBody library.
</p>
</html>"));
end UWInterfaces;
