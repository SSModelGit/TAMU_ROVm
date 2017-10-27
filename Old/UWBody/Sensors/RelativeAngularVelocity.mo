within UWBody.Sensors;

model RelativeAngularVelocity "Measure relative angular velocity between two frame connectors"
  extends Internal.PartialRelativeSensor;
  Modelica.Blocks.Interfaces.RealOutput w_rel[3](each final quantity = "AngularVelocity", each final unit = "rad/s") "Relative angular velocity vector between frame_a and frame_b resolved in frame defined by resolveInFrame" annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {0, -110})));
  UWBody.Interfaces.Frame_resolve frame_resolve if resolveInFrame == UWBody.Types.ResolveInFrameAB.frame_resolve "Coordinate system in which w_rel is optionally resolved" annotation(Placement(transformation(extent = {{84, 64}, {116, 96}}), iconTransformation(extent = {{84, 65}, {116, 97}})));
  parameter UWBody.Types.ResolveInFrameAB resolveInFrame = UWBody.Types.ResolveInFrameAB.frame_a "Frame in which output vector w_rel shall be resolved (world, frame_a, frame_b, or frame_resolve)";
protected
  Internal.BasicRelativeAngularVelocity relativeAngularVelocity(resolveInFrame = resolveInFrame) annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}})));
  UWBody.Interfaces.ZeroPosition zeroPosition if not resolveInFrame == UWBody.Types.ResolveInFrameAB.frame_resolve annotation(Placement(transformation(extent = {{52, 20}, {72, 40}})));
equation
  connect(relativeAngularVelocity.frame_a, frame_a) annotation(Line(points = {{-10, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(relativeAngularVelocity.frame_b, frame_b) annotation(Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(relativeAngularVelocity.frame_resolve, frame_resolve) annotation(Line(points = {{10, 8.1}, {26, 8.1}, {26, 8}, {34, 8}, {34, 80}, {100, 80}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(zeroPosition.frame_resolve, relativeAngularVelocity.frame_resolve) annotation(Line(points = {{52, 30}, {34, 30}, {34, 8.1}, {10, 8.1}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(relativeAngularVelocity.w_rel, w_rel) annotation(Line(points = {{0, -11}, {0, -110}}, color = {0, 0, 127}));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{0, -70}, {0, -110}}, color = {0, 0, 127}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-127, 80}, {134, 120}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{14, -108}, {98, -78}}, textString = "w_rel")}), Documentation(info = "<html>
<p>
The relative angular velocity between frame_a and frame_b is
determined and provided at the output signal connector <b>w_rel</b>.
</p>

<p>
Via parameter <b>resolveInFrame</b> it is defined, in which frame
the angular velocity is resolved:
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
\"frame_resolve\" is enabled and w_rel is resolved in the frame, to
which frame_resolve is connected. Note, if this connector is enabled, it must
be connected.
</p>

<p>
Example: If resolveInFrame = Types.ResolveInFrameAB.frame_a, the output vector is
computed as:
</p>

<pre>
    // Relative orientation object from frame_a to frame_b
    R_rel = MultiBody.Frames.relativeRotation(frame_a.R, frame_b.R);

    // Angular velocity resolved in frame_a
    w_rel = MultiBody.Frames.angularVelocity1(R_rel);
</pre>

</html>"));
end RelativeAngularVelocity;
