within UWBody.Sensors;

model AbsoluteVelocity "Measure absolute velocity vector of origin of frame connector"
  extends Internal.PartialAbsoluteSensor;
  Modelica.Blocks.Interfaces.RealOutput v[3](each final quantity = "Velocity", each final unit = "m/s") "Absolute velocity vector resolved in frame defined by resolveInFrame" annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {110, 0})));
  UWBody.Interfaces.Frame_resolve frame_resolve if resolveInFrame == UWBody.Types.ResolveInFrameA.frame_resolve "Coordinate system in which output vector v is optionally resolved" annotation(Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = -90, origin = {0, -100}), iconTransformation(extent = {{-16, -16}, {16, 16}}, rotation = -90, origin = {0, -100})));
  parameter UWBody.Types.ResolveInFrameA resolveInFrame = UWBody.Types.ResolveInFrameA.frame_a "Frame in which output vector v shall be resolved (world, frame_a, or frame_resolve)";
protected
  Internal.BasicAbsolutePosition position(resolveInFrame = UWBody.Types.ResolveInFrameA.world) annotation(Placement(transformation(extent = {{-60, -10}, {-40, 10}})));
  Modelica.Blocks.Continuous.Der der1[3] annotation(Placement(transformation(extent = {{-20, -20}, {0, 0}}, origin = {10, 10})));
  UWBody.Sensors.TransformAbsoluteVector tansformAbsoluteVector(frame_r_in = UWBody.Types.ResolveInFrameA.world, frame_r_out = resolveInFrame) annotation(Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 90, origin = {50, 0})));
  UWBody.Interfaces.ZeroPosition zeroPosition annotation(Placement(transformation(extent = {{-60, -60}, {-80, -40}})));
  UWBody.Interfaces.ZeroPosition zeroPosition1 if not resolveInFrame == UWBody.Types.ResolveInFrameA.frame_resolve annotation(Placement(transformation(extent = {{60, -60}, {80, -40}})));
equation
  connect(position.r, der1.u) annotation(Line(points = {{-39, 0}, {-12, 0}}, color = {0, 0, 127}));
  connect(position.frame_a, frame_a) annotation(Line(points = {{-60, 0}, {-80, 0}, {-80, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(der1.y, tansformAbsoluteVector.r_in) annotation(Line(points = {{11, 0}, {38, 0}}, color = {0, 0, 127}));
  connect(tansformAbsoluteVector.r_out, v) annotation(Line(points = {{61, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(zeroPosition.frame_resolve, position.frame_resolve) annotation(Line(points = {{-60, -50}, {-50, -50}, {-50, -10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(tansformAbsoluteVector.frame_a, frame_a) annotation(Line(points = {{50, 10}, {50, 20}, {-70, 20}, {-70, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(tansformAbsoluteVector.frame_resolve, zeroPosition1.frame_resolve) annotation(Line(points = {{49.9, -10}, {50, -10}, {50, -50}, {60, -50}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(tansformAbsoluteVector.frame_resolve, frame_resolve) annotation(Line(points = {{49.9, -10}, {50, -10}, {50, -50}, {0, -50}, {0, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{70, 0}, {100, 0}}, color = {0, 0, 127}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-130, 80}, {131, 120}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{58, 18}, {142, 48}}, textString = "v"), Text(visible = true, textColor = {64, 64, 64}, extent = {{15, -92}, {146, -67}}, textString = "resolve"), Line(visible = true, points = {{0, -70}, {0, -95}}, color = {95, 95, 95}, pattern = LinePattern.Dot)}), Documentation(info = "<html>
<p>
The absolute velocity vector of the origin of frame_a is
determined and provided at the output signal connector <b>v</b>.
</p>

<p>
Via parameter <b>resolveInFrame</b> it is defined, in which frame
the velocity vector is resolved:
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>resolveInFrame =<br>Types.ResolveInFrameA.</b></th><th><b>Meaning</b></th></tr>
<tr><td valign=\"top\">world</td>
    <td valign=\"top\">Resolve vector in world frame</td></tr>

<tr><td valign=\"top\">frame_a</td>
    <td valign=\"top\">Resolve vector in frame_a</td></tr>

<tr><td valign=\"top\">frame_resolve</td>
    <td valign=\"top\">Resolve vector in frame_resolve</td></tr>
</table>

<p>
If resolveInFrame = Types.ResolveInFrameA.frame_resolve, the conditional connector
\"frame_resolve\" is enabled and v is resolved in the frame, to
which frame_resolve is connected. Note, if this connector is enabled, it must
be connected.
</p>

<p>
Example: If resolveInFrame = Types.ResolveInFrameA.frame_a, the output vector is
computed as:
</p>

<pre>
    v0 = der(frame_a.r_0);
    v  = MultiBody.Frames.resolve2(frame_a.R, v0);
</pre>

</html>"));
end AbsoluteVelocity;
