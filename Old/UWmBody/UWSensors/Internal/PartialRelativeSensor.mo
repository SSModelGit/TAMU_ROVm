within UWmBody.UWSensors.Internal;

partial model PartialRelativeSensor "Partial relative sensor model for sensors defined by components"
  extends Modelica.Icons.RotationalSensor;
  UWmBody.UWInterfaces.Frame_a frame_a "Coordinate system a" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  UWmBody.UWInterfaces.Frame_b frame_b "Coordinate system b" annotation(Placement(transformation(extent = {{84, -16}, {116, 16}})));
equation
  assert(cardinality(frame_a) > 0, "Connector frame_a must be connected at least once");
  assert(cardinality(frame_b) > 0, "Connector frame_b must be connected at least once");
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-108, 18}, {-72, 43}}, textString = "a"), Text(visible = true, textColor = {64, 64, 64}, extent = {{72, 16}, {108, 41}}, textString = "b"), Line(visible = true, points = {{-70, 0}, {-96, 0}, {-96, 0}}), Line(visible = true, points = {{96, 0}, {70, 0}, {70, 0}}), Line(visible = true, points = {{60, 36}, {60, 36}, {60, 80}, {95, 80}}, pattern = LinePattern.Dot)}));
end PartialRelativeSensor;
