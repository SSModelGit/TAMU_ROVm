within UWmBody.UWSensors.Internal;

model PartialAbsoluteBaseSensor "Partial absolute sensor models for sensors defined by equations (frame_resolve must be connected exactly once)"
  extends Modelica.Icons.RotationalSensor;
  UWmBody.UWInterfaces.Frame_a frame_a "Coordinate system from which kinematic quantities are measured" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  UWmBody.UWInterfaces.Frame_resolve frame_resolve "Coordinate system in which vector is optionally resolved" annotation(Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = -90, origin = {10, -100}), iconTransformation(extent = {{84, -16}, {116, 16}}, rotation = -90, origin = {0, 0}), visible = true));
equation
  assert(cardinality(frame_a) > 0, "Connector frame_a must be connected at least once");
  assert(cardinality(frame_resolve) == 1, "Connector frame_resolve must be connected exactly once");
  frame_a.f = zeros(3);
  frame_a.t = zeros(3);
  frame_resolve.f = zeros(3);
  frame_resolve.t = zeros(3);
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-108, 18}, {-72, 43}}, textString = "a"), Line(visible = true, points = {{-70, 0}, {-96, 0}, {-96, 0}}), Line(visible = true, origin = {85, 0}, rotation = 90, points = {{0, 15}, {0, -15}}, color = {0, 0, 127}), Text(visible = true, textColor = {64, 64, 64}, extent = {{0, -100}, {131, -75}}, textString = "resolve"), Line(visible = true, points = {{0, -100}, {0, -70}, {0, -70}}, pattern = LinePattern.Dot)}));
end PartialAbsoluteBaseSensor;
