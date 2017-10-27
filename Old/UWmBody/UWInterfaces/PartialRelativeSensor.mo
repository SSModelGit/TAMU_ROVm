within UWmBody.UWInterfaces;

partial model PartialRelativeSensor "Base model to measure a relative variable between two frames"
  extends Modelica.Icons.RotationalSensor;
  parameter Integer n_out = 1 "Number of output signals";
  UWInterfaces.Frame_a frame_a "Coordinate system a" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  UWInterfaces.Frame_b frame_b "Coordinate system b" annotation(Placement(transformation(extent = {{84, -16}, {116, 16}})));
  Modelica.Blocks.Interfaces.RealOutput y[n_out] "Measured data as signal vector" annotation(Placement(transformation(origin = {0, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
protected
  outer UWmBody.UWWorld world;
equation
  assert(cardinality(frame_a) > 0, "Connector frame_a of relative sensor object is not connected");
  assert(cardinality(frame_b) > 0, "Connector frame_b of relative sensor object is not connected");
  annotation(Documentation(info = "<html>
<p>
This is a base class for 3-dim. mechanical components with two frames
and one output port in order to measure relative quantities
between the two frames or the cut-forces/torques in the frame and
to provide the measured signals as output for further processing
with the blocks of package Modelica.Blocks.
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{-70, 0}, {-100, 0}}, color = {64, 64, 64}), Line(visible = true, points = {{70, 0}, {100, 0}}, color = {64, 64, 64}), Line(visible = true, points = {{0, -100}, {0, -70}}, color = {0, 36, 164}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-132, 76}, {129, 116}}, textString = "%name"), Text(visible = true, textColor = {128, 128, 128}, extent = {{-118, 27}, {-82, 52}}, textString = "a"), Text(visible = true, textColor = {128, 128, 128}, extent = {{85, 28}, {121, 53}}, textString = "b")}));
end PartialRelativeSensor;
