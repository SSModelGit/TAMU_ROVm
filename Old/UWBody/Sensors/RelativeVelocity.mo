within UWBody.Sensors;

model RelativeVelocity "Measure relative velocity vector between the origins of two frame connectors"
  extends Internal.PartialRelativeSensor;
  Modelica.Blocks.Interfaces.RealOutput v_rel[3](each final quantity = "Velocity", each final unit = "m/s") "Relative velocity vector resolved in frame defined by resolveInFrame" annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {0, -110})));
  UWBody.Interfaces.Frame_resolve frame_resolve if resolveInFrame == UWBody.Types.ResolveInFrameAB.frame_resolve "Coordinate system in which v_rel is optionally resolved" annotation(Placement(transformation(extent = {{84, 64}, {116, 96}}), iconTransformation(extent = {{84, 65}, {116, 97}})));
  parameter UWBody.Types.ResolveInFrameAB resolveInFrame = UWBody.Types.ResolveInFrameAB.frame_a "Frame in which output vector v_rel shall be resolved (world, frame_a, frame_b, or frame_resolve)";
protected
  RelativePosition relativePosition(resolveInFrame = UWBody.Types.ResolveInFrameAB.frame_a) annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}})));
  UWBody.Interfaces.ZeroPosition zeroPosition if not resolveInFrame == UWBody.Types.ResolveInFrameAB.frame_resolve annotation(Placement(transformation(extent = {{50, -60}, {70, -40}})));
  Modelica.Blocks.Continuous.Der der_r_rel[3] annotation(Placement(transformation(extent = {{-20, -20}, {0, 0}}, rotation = -90, origin = {10, -40})));
  UWBody.Sensors.TransformRelativeVector tansformRelativeVector(frame_r_in = UWBody.Types.ResolveInFrameAB.frame_a, frame_r_out = resolveInFrame) annotation(Placement(transformation(extent = {{-10, -80}, {10, -60}})));
equation
  connect(relativePosition.frame_a, frame_a) annotation(Line(points = {{-10, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(relativePosition.frame_b, frame_b) annotation(Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(relativePosition.r_rel, der_r_rel.u) annotation(Line(points = {{0, -11}, {0, -18}}, color = {0, 0, 127}));
  connect(der_r_rel.y, tansformRelativeVector.r_in) annotation(Line(points = {{0, -41}, {0, -58}}, color = {0, 0, 127}));
  connect(tansformRelativeVector.r_out, v_rel) annotation(Line(points = {{0, -81}, {0, -110}}, color = {0, 0, 127}));
  connect(tansformRelativeVector.frame_a, frame_a) annotation(Line(points = {{-10, -70}, {-70, -70}, {-70, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(tansformRelativeVector.frame_b, frame_b) annotation(Line(points = {{10, -70}, {80, -70}, {80, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(tansformRelativeVector.frame_resolve, frame_resolve) annotation(Line(points = {{10, -61.9}, {35, -61.9}, {35, 80}, {100, 80}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(zeroPosition.frame_resolve, tansformRelativeVector.frame_resolve) annotation(Line(points = {{50, -50}, {35, -50}, {35, -61.9}, {10, -61.9}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{0, -70}, {0, -110}}, color = {0, 0, 127}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-127, 80}, {134, 120}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{18, -110}, {102, -80}}, textString = "v_rel")}), Documentation(info = "<html>
<p>
The relative velocity vector between the origins of frame_a and of frame_b are
determined and provided at the output signal connector <b>v_rel</b>.
This vector is defined as:
</p>

<pre>
    r_rel = MultiBody.Frames.resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
    v_rel = <b>der</b>(r_rel);
</pre>

<p>
Via parameter <b>resolveInFrame</b> it is defined, in which frame
the velocity vector is resolved:
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
\"frame_resolve\" is enabled and v_rel is resolved in the frame, to
which frame_resolve is connected. Note, if this connector is enabled, it must
be connected.
</p>

<p>
Example: If resolveInFrame = Types.ResolveInFrameAB.frame_b, the output vector is
computed as:
</p>

<pre>
    r_rel   = MultiBody.Frames.resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
    v_rel_a = <b>der</b>(r_rel);
    v_rel   = MultiBody.Frames.resolveRelative(frame_a.R, frame_b.R, v_rel_a);
</pre>

</html>"));
end RelativeVelocity;
