within UWmBody.UWSensors;

model CutTorque "Measure cut torque vector"
  Modelica.Blocks.Interfaces.RealOutput torque[3] "Cut torque resolved in frame defined by resolveInFrame" annotation(Placement(transformation(origin = {-80, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  parameter Boolean animation = true "= true, if animation shall be enabled (show arrow)";
  parameter Boolean positiveSign = true "= true, if torque with positive sign is returned (= frame_a.t), otherwise with negative sign (= frame_b.t)";
  input Real Nm_to_m(unit = "N.m/m") = 1000 "Torque arrow scaling (length = torque/Nm_to_m)" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Diameter torqueDiameter = world.defaultArrowDiameter "Diameter of torque arrow" annotation(Dialog(group = "if animation = true", enable = animation));
  input UWTypes.Color torqueColor = UWmBody.UWTypes.Defaults.TorqueColor "Color of torque arrow" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "if animation = true", enable = animation));
  extends UWmBody.UWSensors.Internal.PartialCutForceSensor;
protected
  SI.Position t_in_m[3] = frame_a.t * (if positiveSign then +1 else -1) / Nm_to_m "Torque mapped from Nm to m for animation";
  UWVisualizers.Advanced.DoubleArrow torqueArrow(diameter = torqueDiameter, color = torqueColor, specularCoefficient = specularCoefficient, R = frame_b.R, r = frame_b.r_0, r_tail = t_in_m, r_head = -t_in_m) if world.enableAnimation and animation;
  Internal.BasicCutTorque cutTorque(resolveInFrame = resolveInFrame, positiveSign = positiveSign) annotation(Placement(transformation(extent = {{-62, -10}, {-42, 10}})));
  UWmBody.UWInterfaces.ZeroPosition zeroPosition if not resolveInFrame == UWmBody.UWTypes.ResolveInFrameA.frame_resolve annotation(Placement(transformation(extent = {{-20, -40}, {0, -20}})));
equation
  connect(cutTorque.frame_a, frame_a) annotation(Line(points = {{-62, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(cutTorque.frame_b, frame_b) annotation(Line(points = {{-42, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(cutTorque.torque, torque) annotation(Line(points = {{-60, -11}, {-60, -80}, {-80, -80}, {-80, -110}}, color = {0, 0, 127}));
  connect(cutTorque.frame_resolve, frame_resolve) annotation(Line(points = {{-44, -10}, {-44, -74}, {80, -74}, {80, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(zeroPosition.frame_resolve, cutTorque.frame_resolve) annotation(Line(points = {{-20, -30}, {-44, -30}, {-44, -10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-180, -98}, {-64, -72}}, textString = "torque"), Line(visible = true, points = {{-80, -110}, {-80, 0}}, color = {0, 0, 127})}), Documentation(info = "<html>
<p>
The cut-torque acting between the two frames to which this
model is connected, is determined and provided at the output signal connector
<b>torque</b> (= frame_a.t). If parameter <b>positiveSign</b> =
<b>false</b>, the negative cut-torque is provided (= frame_b.t).
</p>

<p>
Via parameter <b>resolveInFrame</b> it is defined, in which frame
the torque vector is resolved:
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
\"frame_resolve\" is enabled and output torque is resolved in the frame, to
which frame_resolve is connected. Note, if this connector is enabled, it must
be connected.
</p>

<p>
In the following figure the animation of a CutTorque
sensor is shown. The dark blue coordinate system is frame_b,
and the green arrow is the cut torque acting at frame_b and
with negative sign at frame_a.
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Sensors/CutTorque.png\">
</p>
</html>"));
end CutTorque;
