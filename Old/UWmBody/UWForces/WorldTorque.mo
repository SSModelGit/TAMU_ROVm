within UWmBody.UWForces;

model WorldTorque "External torque acting at frame_b, defined by 3 input signals and resolved in frame world, frame_b or frame_resolve"
  extends UWInterfaces.PartialOneFrame_b;
  UWInterfaces.Frame_resolve frame_resolve if resolveInFrame == UWmBody.UWTypes.ResolveInFrameB.frame_resolve "The input signals are optionally resolved in this frame" annotation(Placement(transformation(origin = {0, 100}, extent = {{16, -16}, {-16, 16}}, rotation = 270)));
  Modelica.Blocks.Interfaces.RealInput torque[3](each final quantity = "Torque", each final unit = "N.m") "x-, y-, z-coordinates of torque resolved in frame defined by resolveInFrame" annotation(Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter UWmBody.UWTypes.ResolveInFrameB resolveInFrame = UWmBody.UWTypes.ResolveInFrameB.world "Frame in which input torque is resolved (1: world, 2: frame_b, 3: frame_resolve)";
  parameter Real Nm_to_m(unit = "N.m/m") = world.defaultNm_to_m "Torque arrow scaling (length = torque/Nm_to_m)" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Diameter diameter = world.defaultArrowDiameter "Diameter of torque arrow" annotation(Dialog(group = "if animation = true", enable = animation));
  input UWTypes.Color color = UWmBody.UWTypes.Defaults.TorqueColor "Color of arrow" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "if animation = true", enable = animation));
protected
  SI.Position t_in_m[3] = frame_b.t / Nm_to_m "Torque mapped from Nm to m for animation";
  UWVisualizers.Advanced.DoubleArrow arrow(diameter = diameter, color = color, specularCoefficient = specularCoefficient, R = frame_b.R, r = frame_b.r_0, r_tail = t_in_m, r_head = -t_in_m) if world.enableAnimation and animation;
public
  Internal.BasicWorldTorque basicWorldTorque(resolveInFrame = resolveInFrame) annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}})));
protected
  UWInterfaces.ZeroPosition zeroPosition if not resolveInFrame == UWmBody.UWTypes.ResolveInFrameB.frame_resolve annotation(Placement(transformation(extent = {{20, 10}, {40, 30}})));
equation
  connect(basicWorldTorque.frame_b, frame_b) annotation(Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(basicWorldTorque.torque, torque) annotation(Line(points = {{-12, 0}, {-120, 0}}, color = {0, 0, 127}));
  connect(frame_resolve, basicWorldTorque.frame_resolve) annotation(Line(points = {{0, 100}, {0, 10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(zeroPosition.frame_resolve, basicWorldTorque.frame_resolve) annotation(Line(points = {{20, 20}, {0, 20}, {0, 10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  annotation(defaultComponentName = "torque", Documentation(info = "<html>

<p>
The <b>3</b> signals of the <b>torque</b> connector are interpreted
as the x-, y- and z-coordinates of a <b>torque</b> acting at the frame
connector to which frame_b of this component is attached.
Via parameter <b>resolveInFrame</b> it is defined, in which frame these
coordinates shall be resolved:
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>Types.ResolveInFrameB.</b></th><th><b>Meaning</b></th></tr>
<tr><td valign=\"top\">world</td>
    <td valign=\"top\">Resolve input torque in world frame (= default)</td></tr>

<tr><td valign=\"top\">frame_b</td>
    <td valign=\"top\">Resolve input torque in frame_b</td></tr>

<tr><td valign=\"top\">frame_resolve</td>
    <td valign=\"top\">Resolve input torque in frame_resolve (frame_resolve must be connected)</td></tr>
</table>

<p>
If resolveInFrame = Types.ResolveInFrameB.frame_resolve, the torque coordinates
are with respect to the frame, that is connected to <b>frame_resolve</b>.
</p>

<p>
If torque={100,0,0}, and for all parameters the default setting is used,
then the interpretation is that a torque of 100 N is acting along the positive
x-axis of frame_b.
</p>

<p>
Note, the cut-force in frame_b (frame_b.f) is always set to zero.
Conceptually, a force and torque acts on the world frame in such a way that
the force and torque balance between world.frame_b and frame_b is fulfilled.
For efficiency reasons, this reaction torque is, however, not computed.
</p>

<p>
This torque component is by default visualized as a <b>double arrow</b>
acting at the connector to which it is connected. The diameter
and color of the arrow can be defined via
variables <b>diameter</b> and <b>color</b>. The double arrow points
in the direction defined by the
torque vector. The length of the double arrow is proportional
to the length of the torque vector using parameter
<b>Nm_to_m</b> as scaling factor. For example, if Nm_to_m = 100 Nm/m,
then a torque of 350 Nm is displayed as an arrow of length 3.5 m.
</p>
<p>
An example how to use this model is given in the
following figure:
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/WorldTorque1.png\">
</p>

<p>
This leads to the following animation
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/WorldTorque2.png\">
</p>

</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{0, 95}, {0, 82}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Ellipse(visible = true, origin = {0, -100}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, extent = {{-140, -140}, {140, 140}}, startAngle = 60, endAngle = 120), Polygon(visible = true, origin = {0, -100}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-12.5, 8.75}, {0, 13.75}, {12.5, 8.75}, {12.5, -8.75}, {0, -13.75}, {-12.5, -8.75}}), Polygon(visible = true, origin = {70.382, 21.624}, rotation = 60, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{0, -20}, {-10, 10}, {10, 10}}), Polygon(visible = true, origin = {0, -100}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, points = {{-12.549, -11.229}, {-12.549, 8.771}, {-12.549, 8.771}, {-0.049, 13.771}, {-0.049, 13.771}, {12.451, 8.771}, {12.451, 8.771}, {12.451, -11.229}, {12.451, -11.229}, {14.951, -11.229}, {14.951, -11.229}, {24.661, 9.043}, {17.206, 23.399}, {15.55, 34.167}, {29.265, 88.818}, {44.409, 152.161}, {45.781, 156.611}, {54.892, 162.823}, {60, 175.661}, {55.997, 188.913}, {44.677, 196.23}, {29.63, 194.021}, {21.064, 184.033}, {20.105, 172.348}, {24.799, 162.409}, {23.297, 157.033}, {6.853, 90.448}, {-6.675, 35.271}, {-9.574, 29.059}, {-22.964, 19.948}, {-24.207, 3.521}, {-15.049, -11.229}, {-15.049, -11.229}, {-12.549, -11.229}}, smooth = Smooth.Bezier), Ellipse(visible = true, origin = {0, -100}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{27.5, 164.188}, {52.5, 189.188}}), Polygon(visible = true, origin = {17.312, -12.455}, lineColor = {128, 128, 128}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, points = {{-27.312, -59.815}, {-21.575, -57.545}, {-14.064, -57.545}, {-5.944, -60.627}, {0.146, -66.92}, {0.146, -66.92}, {-3.37, -60.294}, {-4.06, -53.116}, {24.507, 64.424}, {26.537, 69.702}, {26.537, 69.702}, {21.868, 69.702}, {16.923, 70.433}, {13.057, 71.813}, {9.078, 74.371}, {9.078, 74.371}, {9.078, 70.108}, {-19.935, -44.005}, {-22.558, -55.739}, {-27.312, -59.815}}, smooth = Smooth.Bezier), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -160}, {150, -120}}, textString = "%name"), Text(visible = true, textColor = {192, 192, 192}, extent = {{-100, 50}, {30, 80}}, textString = "resolve")}));
end WorldTorque;
