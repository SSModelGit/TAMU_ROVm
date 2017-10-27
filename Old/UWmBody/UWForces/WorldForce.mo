within UWmBody.UWForces;

model WorldForce "External force acting at frame_b, defined by 3 input signals and resolved in frame world, frame_b or frame_resolve"
  extends UWInterfaces.PartialOneFrame_b;
  UWInterfaces.Frame_resolve frame_resolve if resolveInFrame == UWmBody.UWTypes.ResolveInFrameB.frame_resolve "The input signals are optionally resolved in this frame" annotation(Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 270)));
  Modelica.Blocks.Interfaces.RealInput force[3](each final quantity = "Force", each final unit = "N") "x-, y-, z-coordinates of force resolved in frame defined by resolveInFrame" annotation(Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter UWmBody.UWTypes.ResolveInFrameB resolveInFrame = UWmBody.UWTypes.ResolveInFrameB.world "Frame in which input force is resolved (1: world, 2: frame_b, 3: frame_resolve)";
  parameter Real N_to_m(unit = "N/m") = world.defaultN_to_m "Force arrow scaling (length = force/N_to_m)" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Diameter diameter = world.defaultArrowDiameter "Diameter of force arrow" annotation(Dialog(group = "if animation = true", enable = animation));
  input UWTypes.Color color = UWmBody.UWTypes.Defaults.ForceColor "Color of arrow" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "if animation = true", enable = animation));
protected
  SI.Position f_in_m[3] = frame_b.f / N_to_m "Force mapped from N to m for animation";
  UWVisualizers.Advanced.Arrow arrow(diameter = diameter, color = color, specularCoefficient = specularCoefficient, R = frame_b.R, r = frame_b.r_0, r_tail = f_in_m, r_head = -f_in_m) if world.enableAnimation and animation;
public
  Internal.BasicWorldForce basicWorldForce(resolveInFrame = resolveInFrame) annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}})));
protected
  UWInterfaces.ZeroPosition zeroPosition if not resolveInFrame == UWmBody.UWTypes.ResolveInFrameB.frame_resolve annotation(Placement(transformation(extent = {{20, -40}, {40, -20}})));
equation
  connect(basicWorldForce.frame_b, frame_b) annotation(Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(basicWorldForce.force, force) annotation(Line(points = {{-12, 0}, {-120, 0}}, color = {0, 0, 127}));
  connect(basicWorldForce.frame_resolve, frame_resolve) annotation(Line(points = {{0, -10}, {0, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(zeroPosition.frame_resolve, basicWorldForce.frame_resolve) annotation(Line(points = {{20, -30}, {0, -30}, {0, -10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  annotation(defaultComponentName = "force", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {5, 5}), graphics = {Text(visible = true, textColor = {192, 192, 192}, extent = {{-100, -76}, {100, -46}}, textString = "resolve"), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{-100, 10}, {40, 10}, {40, 25}, {94, 0}, {40, -25}, {40, -10}, {-100, -10}, {-100, 10}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 45}, {150, 85}}, textString = "%name"), Line(visible = true, points = {{0, -10}, {0, -95}}, color = {95, 95, 95}, pattern = LinePattern.Dot)}), Documentation(info = "<html>

<p>
The <b>3</b> signals of the <b>force</b> connector are interpreted
as the x-, y- and z-coordinates of a <b>force</b> acting at the frame
connector to which frame_b of this component is attached.
Via parameter <b>resolveInFrame</b> it is defined, in which frame these
coordinates shall be resolved:
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>Types.ResolveInFrameB.</b></th><th><b>Meaning</b></th></tr>
<tr><td valign=\"top\">world</td>
    <td valign=\"top\">Resolve input force in world frame (= default)</td></tr>

<tr><td valign=\"top\">frame_b</td>
    <td valign=\"top\">Resolve input force in frame_b</td></tr>

<tr><td valign=\"top\">frame_resolve</td>
    <td valign=\"top\">Resolve input force in frame_resolve (frame_resolve must be connected)</td></tr>
</table>

<p>
If resolveInFrame = Types.ResolveInFrameB.frame_resolve, the force coordinates
are with respect to the frame, that is connected to <b>frame_resolve</b>.
</p>

<p>
If force={100,0,0}, and for all parameters the default setting is used,
then the interpretation is that a force of 100 N is acting along the positive
x-axis of frame_b.
</p>

<p>
Note, the cut-torque in frame_b (frame_b.t) is always set to zero.
Conceptually, a force and torque acts on the world frame in such a way that
the force and torque balance between world.frame_b and frame_b is fulfilled.
For efficiency reasons, this reaction torque is, however, not computed.
</p>

<p>
This force component is by default visualized as an arrow
acting at the connector to which it is connected. The diameter
and color of the arrow can be defined via
variables <b>diameter</b> and <b>color</b>. The arrow
points in the direction defined by the
force signal. The length of the arrow is proportional
to the length of the force vector using parameter
<b>N_to_m</b> as scaling factor. For example, if N_to_m = 100 N/m,
then a force of 350 N is displayed as an arrow of length 3.5 m.
</p>
<p>
An example how to use this model is given in the
following figure:
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/WorldForce1.png\">
</p>

<p>
This leads to the following animation
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/WorldForce2.png\">
</p>

</html>"));
end WorldForce;
