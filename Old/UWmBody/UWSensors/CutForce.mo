within UWmBody.UWSensors;

model CutForce "Measure cut force vector"
  Modelica.Blocks.Interfaces.RealOutput force[3](each final quantity = "Force", each final unit = "N") "Cut force resolved in frame defined by resolveInFrame" annotation(Placement(transformation(origin = {-80, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90), visible = true, iconTransformation(origin = {-80, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  parameter Boolean animation = true "= true, if animation shall be enabled (show arrow)";
  parameter Boolean positiveSign = true "= true, if force with positive sign is returned (= frame_a.f), otherwise with negative sign (= frame_b.f)";
  input Real N_to_m(unit = "N/m") = 1000 "Force arrow scaling (length = force/N_to_m)" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Diameter forceDiameter = world.defaultArrowDiameter "Diameter of force arrow" annotation(Dialog(group = "if animation = true", enable = animation));
  input UWTypes.Color forceColor = UWmBody.UWTypes.Defaults.ForceColor "Color of force arrow" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "if animation = true", enable = animation));
  extends UWmBody.UWSensors.Internal.PartialCutForceSensor;
protected
  SI.Position f_in_m[3] = frame_a.f * (if positiveSign then +1 else -1) / N_to_m "Force mapped from N to m for animation";
  UWVisualizers.Advanced.Arrow forceArrow(diameter = forceDiameter, color = forceColor, specularCoefficient = specularCoefficient, R = frame_b.R, r = frame_b.r_0, r_tail = f_in_m, r_head = -f_in_m) if world.enableAnimation and animation;
  Internal.BasicCutForce cutForce(resolveInFrame = resolveInFrame, positiveSign = positiveSign) annotation(Placement(transformation(extent = {{-50, -10}, {-30, 10}})));
  UWmBody.UWInterfaces.ZeroPosition zeroPosition if not resolveInFrame == UWmBody.UWTypes.ResolveInFrameA.frame_resolve annotation(Placement(transformation(extent = {{0, -40}, {20, -20}})));
equation
  connect(cutForce.frame_a, frame_a) annotation(Line(points = {{-50, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(cutForce.frame_b, frame_b) annotation(Line(points = {{-30, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(cutForce.frame_resolve, frame_resolve) annotation(Line(points = {{-32, -10}, {-32, -60}, {80, -60}, {80, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(cutForce.force, force) annotation(Line(points = {{-48, -11}, {-48, -60}, {-80, -60}, {-80, -110}}, color = {0, 0, 127}, visible = true));
  connect(zeroPosition.frame_resolve, cutForce.frame_resolve) annotation(Line(points = {{0, -30}, {-32, -30}, {-32, -10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-190, -96}, {-74, -70}}, textString = "force"), Line(visible = true, points = {{-80, -110}, {-80, 0}}, color = {0, 0, 127})}), Documentation(info = "<html>
<p>
The cut-force acting between the two frames to which this
model is connected, is determined and provided at the output signal connector
<b>force</b> (= frame_a.f). If parameter <b>positiveSign</b> =
<b>false</b>, the negative cut-force is provided (= frame_b.f).
</p>
<p>
Via parameter <b>resolveInFrame</b> it is defined, in which frame
the force vector is resolved:
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
\"frame_resolve\" is enabled and output force is resolved in the frame, to
which frame_resolve is connected. Note, if this connector is enabled, it must
be connected.
</p>

<p>
In the following figure the animation of a CutForce
sensor is shown. The dark blue coordinate system is frame_b,
and the green arrow is the cut force acting at frame_b and
with negative sign at frame_a.
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Sensors/CutForce.png\">
</p>
</html>"));
end CutForce;
