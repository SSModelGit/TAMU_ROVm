within UWmBody.UWSensors.Internal;

model PartialRelativeBaseSensor "Partial relative sensor models for sensors defined by equations (frame_resolve must be connected exactly once)"
  extends Modelica.Icons.RotationalSensor;
  UWmBody.UWInterfaces.Frame_a frame_a "Coordinate system a (measurement is between frame_a and frame_b)" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  UWmBody.UWInterfaces.Frame_b frame_b "Coordinate system b (measurement is between frame_a and frame_b)" annotation(Placement(transformation(extent = {{84, -16}, {116, 16}})));
  UWmBody.UWInterfaces.Frame_resolve frame_resolve "Coordinate system in which vector is optionally resolved" annotation(Placement(transformation(extent = {{84, 64}, {116, 96}}), iconTransformation(extent = {{84, 65}, {116, 97}})));
equation
  assert(cardinality(frame_a) > 0, "Connector frame_a must be connected at least once");
  assert(cardinality(frame_b) > 0, "Connector frame_b must be connected at least once");
  assert(cardinality(frame_resolve) == 1, "Connector frame_resolve must be connected exactly once");
  frame_a.f = zeros(3);
  frame_a.t = zeros(3);
  frame_b.f = zeros(3);
  frame_b.t = zeros(3);
  frame_resolve.f = zeros(3);
  frame_resolve.t = zeros(3);
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-108, 18}, {-72, 43}}, textString = "a"), Text(visible = true, textColor = {64, 64, 64}, extent = {{72, 16}, {108, 41}}, textString = "b"), Line(visible = true, points = {{-70, 0}, {-96, 0}, {-96, 0}}), Line(visible = true, points = {{96, 0}, {70, 0}, {70, 0}}), Line(visible = true, points = {{0, -70}, {0, -110}}, color = {0, 0, 127}), Line(visible = true, points = {{60, 36}, {60, 36}, {60, 80}, {95, 80}}, pattern = LinePattern.Dot)}));
end PartialRelativeBaseSensor;
