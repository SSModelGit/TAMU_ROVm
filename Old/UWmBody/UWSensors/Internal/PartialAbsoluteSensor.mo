within UWmBody.UWSensors.Internal;

partial model PartialAbsoluteSensor "Partial absolute sensor model for sensors defined by components"
  extends Modelica.Icons.RotationalSensor;
  UWmBody.UWInterfaces.Frame_a frame_a "Coordinate system at which the kinematic quantities are measured" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
equation
  assert(cardinality(frame_a) > 0, "Connector frame_a must be connected at least once");
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-108, 18}, {-72, 43}}, textString = "a"), Line(visible = true, points = {{-70, 0}, {-96, 0}, {-96, 0}})}));
end PartialAbsoluteSensor;
