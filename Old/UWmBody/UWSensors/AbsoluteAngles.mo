within UWmBody.UWSensors;

model AbsoluteAngles "Measure absolute angles between frame connector and the world frame"
  extends Modelica.Icons.RotationalSensor;
  UWmBody.UWInterfaces.Frame_a frame_a "Coordinate system a from which the angles shall be determined" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  Modelica.Blocks.Interfaces.RealOutput angles[3](each final quantity = "Angle", each final unit = "rad", each displayUnit = "deg") "Angles to rotate world frame into frame_a via 'sequence'" annotation(Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));
  parameter UWmBody.UWTypes.RotationSequence sequence(min = {1, 1, 1}, max = {3, 3, 3}) = {1, 2, 3} "Angles are returned to rotate world frame around axes sequence[1], sequence[2] and finally sequence[3] into frame_a" annotation(Evaluate = true);
  parameter Modelica.SIunits.Angle guessAngle1 = 0 "Select angles[1] such that abs(angles[1] - guessAngle1) is a minimum";
equation
  frame_a.f = zeros(3);
  frame_a.t = zeros(3);
  angles = UWFrames.axesRotationsAngles(frame_a.R, sequence, guessAngle1);
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-108, 18}, {-72, 43}}, textString = "a"), Line(visible = true, points = {{-70, 0}, {-96, 0}, {-96, 0}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-132, 80}, {129, 120}}, textString = "%name"), Line(visible = true, points = {{70, 0}, {100, 0}}, color = {0, 0, 127}), Text(visible = true, textColor = {64, 64, 64}, extent = {{62, -44}, {172, -22}}, textString = "angles")}), Documentation(info = "<html>
<p>
This model determines the 3 angles to rotate the world frame
into frame_a along the axes defined by parameter <b>sequence</b>.
For example, if sequence = {3,1,2} then the world frame is
rotated around angles[1] along the z-axis, afterwards it is rotated
around angles[2] along the x-axis, and finally it is rotated around
angles[3] along the y-axis and is then identical to frame_a.
The 3 angles are returned in the range
</p>
<pre>
    -<font face=\"Symbol\">p</font> &lt;= angles[i] &lt;= <font face=\"Symbol\">p</font>
</pre>
<p>
There are <b>two solutions</b> for \"angles[1]\" in this range.
Via parameter <b>guessAngle1</b> (default = 0) the
returned solution is selected such that |angles[1] - guessAngle1| is
minimal. The transformation matrix between the world frame and
frame_a may be in a singular configuration with respect to \"sequence\", i.e.,
there is an infinite number of angle values leading to the same relative
transformation matrix. In this case, the returned solution is
selected by setting angles[1] = guessAngle1. Then angles[2]
and angles[3] can be uniquely determined in the above range.
</p>
<p>
The parameter <b>sequence</b> has the restriction that
only values 1,2,3 can be used and that sequence[1] &ne; sequence[2]
and sequence[2] &ne; sequence[3]. Often used values are:
</p>
<pre>
  sequence = <b>{1,2,3}</b>  // Cardan or Tait-Bryan angle sequence
           = <b>{3,1,3}</b>  // Euler angle sequence
           = <b>{3,2,1}</b>
</pre>
</html>"));
end AbsoluteAngles;
