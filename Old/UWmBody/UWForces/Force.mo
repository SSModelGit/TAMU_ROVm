within UWmBody.UWForces;

model Force "Force acting between two frames, defined by 3 input signals and resolved in frame world, frame_a, frame_b or frame_resolve"
  import Modelica.SIunits.Conversions.to_unit1;
  extends UWmBody.UWInterfaces.PartialTwoFrames;
  UWInterfaces.Frame_resolve frame_resolve if resolveInFrame == UWmBody.UWTypes.ResolveInFrameAB.frame_resolve "The input signals are optionally resolved in this frame" annotation(Placement(transformation(origin = {40, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput force[3](each final quantity = "Force", each final unit = "N") "x-, y-, z-coordinates of force resolved in frame defined by resolveInFrame" annotation(Placement(transformation(origin = {-60, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter UWmBody.UWTypes.ResolveInFrameAB resolveInFrame = UWmBody.UWTypes.ResolveInFrameAB.frame_b "Frame in which input force is resolved (1: world, 2: frame_a, 3: frame_b, 4: frame_resolve)";
  parameter Real N_to_m(unit = "N/m") = world.defaultN_to_m "Force arrow scaling (length = force/N_to_m)" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Diameter forceDiameter = world.defaultArrowDiameter "Diameter of force arrow" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Diameter connectionLineDiameter = forceDiameter "Diameter of line connecting frame_a and frame_b" annotation(Dialog(group = "if animation = true", enable = animation));
  input UWTypes.Color forceColor = UWmBody.UWTypes.Defaults.ForceColor "Color of force arrow" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input UWTypes.Color connectionLineColor = UWmBody.UWTypes.Defaults.SensorColor "Color of line connecting frame_a and frame_b" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "if animation = true", enable = animation));
protected
  SI.Position f_in_m[3] = frame_b.f / N_to_m "Force mapped from N to m for animation";
  UWVisualizers.Advanced.Arrow forceArrow(diameter = forceDiameter, color = forceColor, specularCoefficient = specularCoefficient, R = frame_b.R, r = frame_b.r_0, r_tail = f_in_m, r_head = -f_in_m) if world.enableAnimation and animation;
  UWVisualizers.Advanced.Shape connectionLine(shapeType = "cylinder", lengthDirection = to_unit1(basicForce.r_0), widthDirection = {0, 1, 0}, length = Modelica.Math.Vectors.length(basicForce.r_0), width = connectionLineDiameter, height = connectionLineDiameter, color = connectionLineColor, specularCoefficient = specularCoefficient, r = frame_a.r_0) if world.enableAnimation and animation;
public
  MultiBody.Forces.Internal.BasicForce basicForce(resolveInFrame = resolveInFrame) annotation(Placement(transformation(extent = {{0, -10}, {20, 10}})));
protected
  MultiBody.Interfaces.ZeroPosition zeroPosition if not resolveInFrame == UWmBody.UWTypes.ResolveInFrameAB.frame_resolve annotation(Placement(transformation(extent = {{40, 10}, {60, 30}})));
equation
  connect(basicForce.frame_a, frame_a) annotation(Line(points = {{0, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(basicForce.frame_b, frame_b) annotation(Line(points = {{20, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(force, basicForce.force) annotation(Line(points = {{-60, 120}, {-60, 40}, {4, 40}, {4, 12}}, color = {0, 0, 127}));
  connect(basicForce.frame_resolve, frame_resolve) annotation(Line(points = {{14, 10}, {14, 40}, {40, 40}, {40, 100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(zeroPosition.frame_resolve, basicForce.frame_resolve) annotation(Line(points = {{40, 20}, {27, 20}, {27, 10}, {14, 10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-98, -98}, {99, 99}}), Text(visible = true, textColor = {192, 192, 192}, extent = {{-92, 35}, {87, 61}}, textString = "resolve"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -95}, {150, -55}}, textString = "%name"), Line(visible = true, points = {{40, 100}, {40, 0}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Line(visible = true, points = {{-60, 100}, {40, 100}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Line(visible = true, points = {{20, 0}, {75, 0}}, color = {64, 64, 64}, thickness = 10, arrowSize = 30), Line(visible = true, points = {{-70, 0}, {-20, 0}}, color = {64, 64, 64}, thickness = 10, arrowSize = 30), Line(visible = true, points = {{20, 0}, {95, 0}}, color = {64, 64, 64}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 40), Line(visible = true, points = {{-95, 0}, {-20, 0}}, color = {64, 64, 64}, arrow = {Arrow.Filled, Arrow.None}, arrowSize = 40)}), Documentation(info = "<html>
<p>
The <b>3</b> signals of the <b>force</b> connector are interpreted
as the x-, y- and z-coordinates of a <b>force</b> acting at the frame
connector to which frame_b of this component is attached.
Via parameter <b>resolveInFrame</b> it is defined, in which frame these
coordinates shall be resolved:
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>Types.ResolveInFrameAB.</b></th><th><b>Meaning</b></th></tr>
<tr><td valign=\"top\">world</td>
    <td valign=\"top\">Resolve input force in world frame</td></tr>

<tr><td valign=\"top\">frame_a</td>
    <td valign=\"top\">Resolve input force in frame_a</td></tr>

<tr><td valign=\"top\">frame_b</td>
    <td valign=\"top\">Resolve input force in frame_b (= default)</td></tr>

<tr><td valign=\"top\">frame_resolve</td>
    <td valign=\"top\">Resolve input force in frame_resolve (frame_resolve must be connected)</td></tr>
</table>

<p>
If resolveInFrame = ResolveInFrameAB.frame_resolve, the force coordinates
are with respect to the frame, that is connected to <b>frame_resolve</b>.
</p>

<p>
If force={100,0,0}, and for all parameters the default setting is used,
then the interpretation is that a force of 100 N is acting along the positive
x-axis of frame_b.
</p>

<p>
Note, the cut-torque in frame_b (frame_b.t) is always set to zero.
Additionally, a force and torque acts on frame_a in such a way that
the force and torque balance between frame_a and frame_b is fulfilled.
</p>

<p>
An example how to use this model is given in the
following figure:
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/Force1.png\">
</p>

<p>
This leads to the following animation (the yellow cylinder
characterizes the line between frame_a and frame_b of the
Force component, i.e., the force acts with negative sign
also on the opposite side of this cylinder, but for
clarity this is not shown in the animation):
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/Force2.png\">
</p>

</html>"));
end Force;
