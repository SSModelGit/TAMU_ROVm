within UWmBody.UWForces;

model Torque "Torque acting between two frames, defined by 3 input signals and resolved in frame world, frame_a, frame_b or frame_resolve"
  import Modelica.SIunits.Conversions.to_unit1;
  extends UWmBody.UWInterfaces.PartialTwoFrames;
  UWInterfaces.Frame_resolve frame_resolve if resolveInFrame == UWmBody.UWTypes.ResolveInFrameAB.frame_resolve "The input signals are optionally resolved in this frame" annotation(Placement(transformation(origin = {40, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput torque[3](each final quantity = "Torque", each final unit = "N.m") "x-, y-, z-coordinates of torque resolved in frame defined by resolveInFrame" annotation(Placement(transformation(origin = {-60, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter UWmBody.UWTypes.ResolveInFrameAB resolveInFrame = UWmBody.UWTypes.ResolveInFrameAB.frame_b "Frame in which input force is resolved (1: world, 2: frame_a, 3: frame_b, 4: frame_resolve)";
  parameter Real Nm_to_m(unit = "N.m/m") = world.defaultNm_to_m "Torque arrow scaling (length = torque/Nm_to_m)" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Diameter torqueDiameter = world.defaultArrowDiameter "Diameter of torque arrow" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Diameter connectionLineDiameter = torqueDiameter "Diameter of line connecting frame_a and frame_b" annotation(Dialog(group = "if animation = true", enable = animation));
  input UWTypes.Color torqueColor = UWmBody.UWTypes.Defaults.TorqueColor "Color of torque arrow" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input UWTypes.Color connectionLineColor = UWmBody.UWTypes.Defaults.SensorColor "Color of line connecting frame_a and frame_b" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "if animation = true", enable = animation));
protected
  SI.Position t_in_m[3] = frame_b.t / Nm_to_m "Torque mapped from Nm to m for animation";
  UWVisualizers.Advanced.DoubleArrow torqueArrow(diameter = torqueDiameter, color = torqueColor, specularCoefficient = specularCoefficient, R = frame_b.R, r = frame_b.r_0, r_tail = t_in_m, r_head = -t_in_m) if world.enableAnimation and animation;
  UWVisualizers.Advanced.Shape connectionLine(shapeType = "cylinder", lengthDirection = to_unit1(basicTorque.r_0), widthDirection = {0, 1, 0}, length = Modelica.Math.Vectors.length(basicTorque.r_0), width = connectionLineDiameter, height = connectionLineDiameter, color = connectionLineColor, specularCoefficient = specularCoefficient, r = frame_a.r_0) if world.enableAnimation and animation;
public
  Internal.BasicTorque basicTorque(resolveInFrame = resolveInFrame) annotation(Placement(transformation(extent = {{-8, -10}, {12, 10}})));
protected
  UWInterfaces.ZeroPosition zeroPosition if not resolveInFrame == UWmBody.UWTypes.ResolveInFrameAB.frame_resolve annotation(Placement(transformation(extent = {{34, 10}, {54, 30}})));
equation
  connect(basicTorque.frame_a, frame_a) annotation(Line(points = {{-8, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(basicTorque.frame_b, frame_b) annotation(Line(points = {{12, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(basicTorque.torque, torque) annotation(Line(points = {{-4, 12}, {-4, 60}, {-60, 60}, {-60, 120}}, color = {0, 36, 164}, visible = true));
  connect(basicTorque.frame_resolve, frame_resolve) annotation(Line(points = {{6, 10}, {6, 60}, {40, 60}, {40, 100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(zeroPosition.frame_resolve, basicTorque.frame_resolve) annotation(Line(points = {{34, 20}, {20, 20}, {20, 10}, {6, 10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-98, -98}, {99, 99}}), Line(visible = true, points = {{-60, 100}, {40, 100}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Ellipse(visible = true, origin = {0, -100}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, extent = {{-140, -140}, {140, 140}}, startAngle = 60, endAngle = 120), Polygon(visible = true, origin = {0, -100}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-12.5, 8.75}, {0, 13.75}, {12.5, 8.75}, {12.5, -8.75}, {0, -13.75}, {-12.5, -8.75}}), Polygon(visible = true, origin = {0, -100}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, points = {{-12.549, -11.229}, {-12.549, 8.771}, {-12.549, 8.771}, {-0.049, 13.771}, {-0.049, 13.771}, {12.451, 8.771}, {12.451, 8.771}, {12.451, -11.229}, {12.451, -11.229}, {14.951, -11.229}, {14.951, -11.229}, {24.661, 9.043}, {17.206, 23.399}, {15.55, 34.167}, {29.265, 88.818}, {44.409, 152.161}, {45.781, 156.611}, {54.892, 162.823}, {60, 175.661}, {55.997, 188.913}, {44.677, 196.23}, {29.63, 194.021}, {21.064, 184.033}, {20.105, 172.348}, {24.799, 162.409}, {23.297, 157.033}, {6.853, 90.448}, {-6.675, 35.271}, {-9.574, 29.059}, {-22.964, 19.948}, {-24.207, 3.521}, {-15.049, -11.229}, {-15.049, -11.229}, {-12.549, -11.229}}, smooth = Smooth.Bezier), Ellipse(visible = true, origin = {0, -100}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{27.5, 164.188}, {52.5, 189.188}}), Polygon(visible = true, origin = {17.312, -12.455}, lineColor = {128, 128, 128}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, points = {{-27.312, -59.815}, {-21.575, -57.545}, {-14.064, -57.545}, {-5.944, -60.627}, {0.146, -66.92}, {0.146, -66.92}, {-3.37, -60.294}, {-4.06, -53.116}, {24.507, 64.424}, {26.537, 69.702}, {26.537, 69.702}, {21.868, 69.702}, {16.923, 70.433}, {13.057, 71.813}, {9.078, 74.371}, {9.078, 74.371}, {9.078, 70.108}, {-19.935, -44.005}, {-22.558, -55.739}, {-27.312, -59.815}}, smooth = Smooth.Bezier), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -160}, {150, -120}}, textString = "%name"), Text(visible = true, textColor = {192, 192, 192}, extent = {{-100, 50}, {30, 80}}, textString = "resolve"), Polygon(visible = true, rotation = 60, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{53.392, -68.478}, {43.392, -38.478}, {63.392, -38.478}}), Polygon(visible = true, rotation = -60, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{-53.392, -68.478}, {-63.392, -38.478}, {-43.392, -38.478}})}), Documentation(info = "<html>
<p>
The <b>3</b> signals of the <b>torque</b> connector are interpreted
as the x-, y- and z-coordinates of a <b>torque</b> acting at the frame
connector to which frame_b of this component is attached.
Via parameter <b>resolveInFrame</b> it is defined, in which frame these
coordinates shall be resolved:
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>Types.ResolveInFrameAB.</b></th><th><b>Meaning</b></th></tr>
<tr><td valign=\"top\">world</td>
    <td valign=\"top\">Resolve input torque in world frame</td></tr>

<tr><td valign=\"top\">frame_a</td>
    <td valign=\"top\">Resolve input torque in frame_a</td></tr>

<tr><td valign=\"top\">frame_b</td>
    <td valign=\"top\">Resolve input torque in frame_b (= default)</td></tr>

<tr><td valign=\"top\">frame_resolve</td>
    <td valign=\"top\">Resolve input torque in frame_resolve (frame_resolve must be connected)</td></tr>
</table>

<p>
If resolveInFrame = ResolveInFrameAB.frame_resolve, the torque coordinates
are with respect to the frame, that is connected to <b>frame_resolve</b>.
</p>

<p>
If torque={100,0,0}, and for all parameters the default setting is used,
then the interpretation is that a torque of 100 N.m is acting along the positive
x-axis of frame_b.
</p>

<p>
Note, the cut-forces in frame_a and frame_b (frame_a.f, frame_b.f) are
always set to zero and the cut-torque at frame_a (frame_a.t) is the same
as the cut-torque at frame_b (frame_b.t) but with opposite sign.
</p>

<p>
An example how to use this model is given in the
following figure:
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/Torque1.png\">
</p>

<p>
This leads to the following animation (the yellow cylinder
characterizes the line between frame_a and frame_b of the
Torque component, i.e., the torque acts with negative sign
also on the opposite side of this cylinder, but for
clarity this is not shown in the animation):
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/Torque2.png\">
</p>

</html>"));
end Torque;
