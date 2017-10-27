within UWmBody.UWInterfaces;

partial model PartialAbsoluteSensor "Base model to measure an absolute frame variable"
  extends Modelica.Icons.RotationalSensor;
  parameter Integer n_out = 1 "Number of output signals";
  UWInterfaces.Frame_a frame_a "Coordinate system from which absolute quantities are provided as output signals" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  Modelica.Blocks.Interfaces.RealOutput y[n_out] "Measured data as signal vector" annotation(Placement(transformation(extent = {{100, -10}, {120, 10}})));
protected
  outer UWmBody.UWWorld world;
equation
  assert(cardinality(frame_a) > 0, "Connector frame_a of absolute sensor object is not connected");
  annotation(Documentation(info = "<html>
<p>
This is the base class of a 3-dim. mechanics component with one frame and one
output port in order to measure an absolute quantity in the frame connector
and to provide the measured signal as output for further processing
with the blocks of package Modelica.Blocks.
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{-70, 0}, {-100, 0}}), Line(visible = true, points = {{70, 0}, {100, 0}}, color = {0, 0, 255}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-132, -120}, {131, -80}}, textString = "%name")}));
end PartialAbsoluteSensor;
