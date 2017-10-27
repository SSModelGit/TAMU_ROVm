within UWmBody.UWSensors;

model AbsoluteAngularVelocity "Measure absolute angular velocity of frame connector"
  extends Internal.PartialAbsoluteSensor;
  Modelica.Blocks.Interfaces.RealOutput w[3](each final quantity = "AngularVelocity", each final unit = "rad/s") "Absolute angular velocity vector of frame_a with respect to world frame, resolved in frame defined by resolveInFrame" annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {110, 0})));
  UWmBody.UWInterfaces.Frame_resolve frame_resolve if resolveInFrame == UWmBody.UWTypes.ResolveInFrameA.frame_resolve "Coordinate system in which w is optionally resolved" annotation(Placement(transformation(extent = {{-16, -16}, {16, 16}}, rotation = -90, origin = {0, -100}), iconTransformation(extent = {{-16, -16}, {16, 16}}, rotation = -90, origin = {0, -101})));
  parameter UWmBody.UWTypes.ResolveInFrameA resolveInFrame = UWmBody.UWTypes.ResolveInFrameA.frame_a "Frame in which output vector w shall be resolved (world, frame_a, or frame_resolve)";
protected
  Internal.BasicAbsoluteAngularVelocity angularVelocity(resolveInFrame = resolveInFrame) annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}})));
  UWmBody.UWInterfaces.ZeroPosition zeroPosition if not resolveInFrame == UWmBody.UWTypes.ResolveInFrameA.frame_resolve annotation(Placement(transformation(extent = {{40, -40}, {60, -20}})));
equation
  connect(angularVelocity.frame_resolve, frame_resolve) annotation(Line(points = {{0, -10}, {0, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(zeroPosition.frame_resolve, angularVelocity.frame_resolve) annotation(Line(points = {{40, -30}, {0, -30}, {0, -10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(angularVelocity.w, w) annotation(Line(points = {{11, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(angularVelocity.frame_a, frame_a) annotation(Line(points = {{-10, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{70, 0}, {100, 0}}, color = {0, 0, 127}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-127, 80}, {134, 120}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{66, 18}, {150, 48}}, textString = "w"), Line(visible = true, points = {{0, -70}, {0, -96}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Text(visible = true, textColor = {64, 64, 64}, extent = {{15, -92}, {146, -67}}, textString = "resolve")}), Documentation(info = "<html>
<p>
The absolute angular velocity of frame_a with respect to the
world frame is
determined and provided at the output signal connector <b>w</b>.
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

<tr><td valign=\"top\">frame_resolve</td>
    <td valign=\"top\">Resolve vector in frame_resolve</td></tr>
</table>

<p>
If resolveInFrame = Types.ResolveInFrameA.frame_resolve, the conditional connector
\"frame_resolve\" is enabled and w is resolved in the frame, to
which frame_resolve is connected. Note, if this connector is enabled, it must
be connected.
</p>

<p>
Example: If resolveInFrame = Types.ResolveInFrameA.frame_a, the output vector is
computed as:
</p>

<pre>
    w = MultiBody.Frames.angularVelocity2(frame_a.R);
</pre>

</html>"));
end AbsoluteAngularVelocity;
