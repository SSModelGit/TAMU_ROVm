within UWmBody.UWSensors;

model CutForceAndTorque "Measure cut force and cut torque vector"
  import UWmBody.UWTypes;
  Modelica.Blocks.Interfaces.RealOutput force[3](each final quantity = "Force", each final unit = "N") "Cut force resolved in frame defined by resolveInFrame" annotation(Placement(transformation(origin = {-80, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput torque[3] "Cut torque resolved in frame defined by resolveInFrame" annotation(Placement(transformation(origin = {0, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  parameter Boolean animation = true "= true, if animation shall be enabled (show force and torque arrow)";
  parameter Boolean positiveSign = true "= true, if force and torque with positive sign is returned (= frame_a.f/.t), otherwise with negative sign (= frame_b.f/.t)";
  input Real N_to_m(unit = "N/m") = 1000 "Force arrow scaling (length = force/N_to_m)" annotation(Dialog(group = "if animation = true", enable = animation));
  input Real Nm_to_m(unit = "N.m/m") = 1000 "Torque arrow scaling (length = torque/Nm_to_m)" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Diameter forceDiameter = world.defaultArrowDiameter "Diameter of force arrow" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Diameter torqueDiameter = forceDiameter "Diameter of torque arrow" annotation(Dialog(group = "if animation = true", enable = animation));
  input UWTypes.Color forceColor = UWmBody.UWTypes.Defaults.ForceColor "Color of force arrow" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input UWTypes.Color torqueColor = UWmBody.UWTypes.Defaults.TorqueColor "Color of torque arrow" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "if animation = true", enable = animation));
  extends UWmBody.UWSensors.Internal.PartialCutForceSensor;
protected
  parameter Integer csign = if positiveSign then +1 else -1;
  SI.Position f_in_m[3] = frame_a.f * csign / N_to_m "Force mapped from N to m for animation";
  SI.Position t_in_m[3] = frame_a.t * csign / Nm_to_m "Torque mapped from Nm to m for animation";
  UWVisualizers.Advanced.Arrow forceArrow(diameter = forceDiameter, color = forceColor, specularCoefficient = specularCoefficient, R = frame_b.R, r = frame_b.r_0, r_tail = f_in_m, r_head = -f_in_m) if world.enableAnimation and animation;
  UWVisualizers.Advanced.DoubleArrow torqueArrow(diameter = torqueDiameter, color = torqueColor, specularCoefficient = specularCoefficient, R = frame_b.R, r = frame_b.r_0, r_tail = t_in_m, r_head = -t_in_m) if world.enableAnimation and animation;
  Internal.BasicCutForce cutForce(resolveInFrame = resolveInFrame, positiveSign = positiveSign) annotation(Placement(transformation(extent = {{-60, -10}, {-40, 10}})));
  Internal.BasicCutTorque cutTorque(resolveInFrame = resolveInFrame, positiveSign = positiveSign) annotation(Placement(transformation(extent = {{-2, -10}, {18, 10}})));
  UWmBody.UWInterfaces.ZeroPosition zeroPosition if not resolveInFrame == UWmBody.UWTypes.ResolveInFrameA.frame_resolve annotation(Placement(transformation(extent = {{60, 30}, {80, 50}})));
equation
  connect(cutForce.frame_a, frame_a) annotation(Line(points = {{-60, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(cutForce.frame_b, cutTorque.frame_a) annotation(Line(points = {{-40, 0}, {-2, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(cutTorque.frame_b, frame_b) annotation(Line(points = {{18, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(cutForce.force, force) annotation(Line(points = {{-58, -11}, {-58, -20}, {-80, -20}, {-80, -110}}, color = {0, 0, 127}));
  connect(cutTorque.torque, torque) annotation(Line(points = {{0, -11}, {0, -110}}, color = {0, 0, 127}));
  connect(zeroPosition.frame_resolve, cutTorque.frame_resolve) annotation(Line(points = {{60, 40}, {32, 40}, {32, -20}, {16, -20}, {16, -10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(zeroPosition.frame_resolve, cutForce.frame_resolve) annotation(Line(points = {{60, 40}, {-26, 40}, {-26, -20}, {-42, -20}, {-42, -10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(cutForce.frame_resolve, frame_resolve) annotation(Line(points = {{-42, -10}, {-42, -70}, {80, -70}, {80, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(cutTorque.frame_resolve, frame_resolve) annotation(Line(points = {{16, -10}, {16, -70}, {80, -70}, {80, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{-80, -110}, {-80, 0}}, color = {0, 0, 127}), Line(visible = true, points = {{0, -110}, {0, -70}}, color = {0, 0, 127}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-188, -96}, {-72, -70}}, textString = "force"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-56, -96}, {60, -70}}, textString = "torque")}), Documentation(info = "<html>
<p>
The cut-force and cut-torque acting between the two frames to which this
model is connected, are determined and provided at the output signal connectors
<b>force</b> (= frame_a.f) and <b>torque</b> (= frame_a.t).
If parameter <b>positiveSign</b> =
<b>false</b>, the negative cut-force and cut-torque is provided
(= frame_b.f, frame_b.t).
</p>
<p>
Via parameter <b>resolveInFrame</b> it is defined, in which frame
the two vectors are resolved:
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>resolveInFrame =<br>Types.ResolveInFrameAB.</b></th><th><b>Meaning</b></th></tr>
<tr><td valign=\"top\">world</td>
    <td valign=\"top\">Resolve vectors in world frame</td></tr>

<tr><td valign=\"top\">frame_a</td>
    <td valign=\"top\">Resolve vectors in frame_a</td></tr>

<tr><td valign=\"top\">frame_b</td>
    <td valign=\"top\">Resolve vectors in frame_b</td></tr>

<tr><td valign=\"top\">frame_resolve</td>
    <td valign=\"top\">Resolve vectors in frame_resolve</td></tr>
</table>

<p>
If resolveInFrame = Types.ResolveInFrameAB.frame_resolve, the conditional connector
\"frame_resolve\" is enabled and the output vectors force and torque are resolved in the frame, to
which frame_resolve is connected. Note, if this connector is enabled, it must
be connected.
</p>

<p>
In the following figure the animation of a CutForceAndTorque
sensor is shown. The dark blue coordinate system is frame_b,
and the green arrows are the cut force and the cut torque,
respectively, acting at frame_b and
with negative sign at frame_a.
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Sensors/CutForceAndTorque.png\">
</p>
</html>"));
end CutForceAndTorque;
