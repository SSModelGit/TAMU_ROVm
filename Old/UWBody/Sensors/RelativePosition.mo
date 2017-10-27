within UWBody.Sensors;

model RelativePosition "Measure relative position vector between the origins of two frame connectors"
  extends Internal.PartialRelativeSensor;
  Modelica.Blocks.Interfaces.RealOutput r_rel[3] "Relative position vector resolved in frame defined by resolveInFrame" annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {0, -110})));
  UWBody.Interfaces.Frame_resolve frame_resolve if resolveInFrame == UWBody.Types.ResolveInFrameAB.frame_resolve "Coordinate system in which r_rel is optionally resolved" annotation(Placement(transformation(extent = {{84, 64}, {116, 96}}), iconTransformation(extent = {{84, 65}, {116, 97}})));
  parameter UWBody.Types.ResolveInFrameAB resolveInFrame = UWBody.Types.ResolveInFrameAB.frame_a "Frame in which output vector r_rel shall be resolved (world, frame_a, frame_b, or frame_resolve)";
protected
  Internal.BasicRelativePosition relativePosition(resolveInFrame = resolveInFrame) annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}})));
  UWBody.Interfaces.ZeroPosition zeroPosition if not resolveInFrame == UWBody.Types.ResolveInFrameAB.frame_resolve annotation(Placement(transformation(extent = {{52, 20}, {72, 40}})));
equation
  connect(relativePosition.frame_a, frame_a) annotation(Line(points = {{-10, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(relativePosition.frame_b, frame_b) annotation(Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(relativePosition.frame_resolve, frame_resolve) annotation(Line(points = {{10, 8.1}, {26, 8.1}, {26, 8}, {36, 8}, {36, 80}, {100, 80}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(zeroPosition.frame_resolve, relativePosition.frame_resolve) annotation(Line(points = {{52, 30}, {36, 30}, {36, 8.1}, {10, 8.1}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(relativePosition.r_rel, r_rel) annotation(Line(points = {{0, -11}, {0, -110}}, color = {0, 0, 127}));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{0, -70}, {0, -110}}, color = {0, 0, 127}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-127, 80}, {134, 120}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{18, -110}, {102, -80}}, textString = "r_rel")}), Documentation(info = "<html>
<p>
The relative position vector between the origins of frame_a and frame_b are
determined and provided at the output signal connector <b>r_rel</b>.
</p>

<p>
Via parameter <b>resolveInFrame</b> it is defined, in which frame
the position vector is resolved:
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>resolveInFrame =<br>Types.ResolveInFrameAB.</b></th><th><b>Meaning</b></th></tr>
<tr><td valign=\"top\">world</td>
    <td valign=\"top\">Resolve vector in world frame</td></tr>

<tr><td valign=\"top\">frame_a</td>
    <td valign=\"top\">Resolve vector in frame_a</td></tr>

<tr><td valign=\"top\">frame_b</td>
    <td valign=\"top\">Resolve vector in frame_b</td></tr>

<tr><td valign=\"top\">frame_resolve</td>
    <td valign=\"top\">Resolve vector in frame_resolve</td></tr>
</table>

<p>
If resolveInFrame = Types.ResolveInFrameAB.frame_resolve, the conditional connector
\"frame_resolve\" is enabled and r_rel is resolved in the frame, to
which frame_resolve is connected. Note, if this connector is enabled, it must
be connected.
</p>

<p>
Example: If resolveInFrame = Types.ResolveInFrameAB.frame_a, the output vector is
computed as:
</p>

<pre>
    r_rel = MultiBody.Frames.resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
</pre>
</html>"));
end RelativePosition;
