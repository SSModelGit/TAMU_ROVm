within UWBody.Forces;

model ForceAndTorque "Force and torque acting between two frames, defined by 3+3 input signals and resolved in frame world, frame_a, frame_b or frame_resolve"
  import UWBody.Types;
  import Modelica.SIunits.Conversions.to_unit1;
  extends UWBody.Interfaces.PartialTwoFrames;
  Modelica.Blocks.Interfaces.RealInput force[3](each final quantity = "Force", each final unit = "N") "x-, y-, z-coordinates of force resolved in frame defined by resolveInFrame" annotation(Placement(transformation(origin = {-80, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
  Modelica.Blocks.Interfaces.RealInput torque[3](each final quantity = "Torque", each final unit = "N.m") "x-, y-, z-coordinates of torque resolved in frame defined by resolveInFrame" annotation(Placement(transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
  Interfaces.Frame_resolve frame_resolve if resolveInFrame == UWBody.Types.ResolveInFrameAB.frame_resolve "The input signals are optionally resolved in this frame" annotation(Placement(transformation(origin = {80, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter UWBody.Types.ResolveInFrameAB resolveInFrame = UWBody.Types.ResolveInFrameAB.frame_b "Frame in which input force and torque are resolved (1: world, 2: frame_a, 3: frame_b, 4: frame_resolve)";
  parameter Real N_to_m(unit = "N/m") = world.defaultN_to_m "Force arrow scaling (length = force/N_to_m)" annotation(Dialog(group = "if animation = true", enable = animation));
  parameter Real Nm_to_m(unit = "N.m/m") = world.defaultNm_to_m "Torque arrow scaling (length = torque/Nm_to_m)" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Diameter forceDiameter = world.defaultArrowDiameter "Diameter of force arrow" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Diameter torqueDiameter = forceDiameter "Diameter of torque arrow" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Diameter connectionLineDiameter = forceDiameter "Diameter of line connecting frame_a and frame_b" annotation(Dialog(group = "if animation = true", enable = animation));
  input Types.Color forceColor = UWBody.Types.Defaults.ForceColor "Color of force arrow" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input Types.Color torqueColor = UWBody.Types.Defaults.TorqueColor "Color of torque arrow" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input Types.Color connectionLineColor = UWBody.Types.Defaults.SensorColor "Color of line connecting frame_a and frame_b" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "if animation = true", enable = animation));
protected
  SI.Position f_in_m[3] = frame_b.f / N_to_m "Force mapped from N to m for animation";
  SI.Position t_in_m[3] = frame_b.t / Nm_to_m "Torque mapped from Nm to m for animation";
  Visualizers.Advanced.Arrow forceArrow(diameter = forceDiameter, color = forceColor, specularCoefficient = specularCoefficient, R = frame_b.R, r = frame_b.r_0, r_tail = f_in_m, r_head = -f_in_m) if world.enableAnimation and animation;
  Visualizers.Advanced.DoubleArrow torqueArrow(diameter = torqueDiameter, color = torqueColor, specularCoefficient = specularCoefficient, R = frame_b.R, r = frame_b.r_0, r_tail = t_in_m, r_head = -t_in_m) if world.enableAnimation and animation;
  Visualizers.Advanced.Shape connectionLine(shapeType = "cylinder", lengthDirection = to_unit1(basicForce.r_0), widthDirection = {0, 1, 0}, length = Modelica.Math.Vectors.length(basicForce.r_0), width = connectionLineDiameter, height = connectionLineDiameter, color = connectionLineColor, specularCoefficient = specularCoefficient, r = frame_a.r_0) if world.enableAnimation and animation;
public
  Internal.BasicForce basicForce(resolveInFrame = resolveInFrame) annotation(Placement(transformation(extent = {{-84, -10}, {-64, 10}})));
  Internal.BasicTorque basicTorque(resolveInFrame = resolveInFrame) annotation(Placement(transformation(extent = {{-4, 10}, {16, 30}})));
protected
  Interfaces.ZeroPosition zeroPosition if not resolveInFrame == UWBody.Types.ResolveInFrameAB.frame_resolve annotation(Placement(transformation(extent = {{20, 30}, {40, 50}})));
equation
  connect(basicForce.frame_a, frame_a) annotation(Line(points = {{-84, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(basicForce.frame_b, frame_b) annotation(Line(points = {{-64, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(basicTorque.frame_b, frame_b) annotation(Line(points = {{16, 20}, {68, 20}, {68, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(basicTorque.frame_a, frame_a) annotation(Line(points = {{-4, 20}, {-90, 20}, {-90, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(basicForce.force, force) annotation(Line(points = {{-80, 12}, {-80, 120}}, color = {0, 36, 164}, visible = true));
  connect(basicTorque.torque, torque) annotation(Line(points = {{0, 32}, {0, 120}}, color = {0, 36, 164}, visible = true));
  connect(basicTorque.frame_resolve, frame_resolve) annotation(Line(points = {{10, 30}, {10, 80}, {80, 80}, {80, 100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(basicForce.frame_resolve, frame_resolve) annotation(Line(points = {{-70, 10}, {-70, 80}, {80, 80}, {80, 100}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(zeroPosition.frame_resolve, basicTorque.frame_resolve) annotation(Line(points = {{20, 40}, {10, 40}, {10, 30}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(zeroPosition.frame_resolve, basicForce.frame_resolve) annotation(Line(points = {{20, 40}, {-70, 40}, {-70, 10}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-98, -98}, {99, 99}}), Text(visible = true, textColor = {192, 192, 192}, extent = {{-100, 50}, {30, 80}}, textString = "resolve"), Line(visible = true, points = {{80, 100}, {80, 0}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Line(visible = true, points = {{-80, 100}, {80, 100}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Line(visible = true, points = {{20, 0}, {75, 0}}, color = {64, 64, 64}, thickness = 10, arrowSize = 30), Line(visible = true, points = {{-70, 0}, {-20, 0}}, color = {64, 64, 64}, thickness = 10, arrowSize = 30), Line(visible = true, points = {{20, 0}, {95, 0}}, color = {64, 64, 64}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 40), Line(visible = true, points = {{-95, 0}, {-20, 0}}, color = {64, 64, 64}, arrow = {Arrow.Filled, Arrow.None}, arrowSize = 40), Ellipse(visible = true, origin = {0, -90}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, extent = {{-140, -140}, {140, 140}}, startAngle = 60, endAngle = 120), Polygon(visible = true, origin = {0, -90}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-12.5, 8.75}, {0, 13.75}, {12.5, 8.75}, {12.5, -8.75}, {0, -13.75}, {-12.5, -8.75}}), Polygon(visible = true, origin = {0, -90}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, points = {{-12.549, -11.229}, {-12.549, 8.771}, {-12.549, 8.771}, {-0.049, 13.771}, {-0.049, 13.771}, {12.451, 8.771}, {12.451, 8.771}, {12.451, -11.229}, {12.451, -11.229}, {14.951, -11.229}, {14.951, -11.229}, {24.661, 9.043}, {17.206, 23.399}, {15.55, 34.167}, {29.265, 88.818}, {44.409, 152.161}, {45.781, 156.611}, {54.892, 162.823}, {60, 175.661}, {55.997, 188.913}, {44.677, 196.23}, {29.63, 194.021}, {21.064, 184.033}, {20.105, 172.348}, {24.799, 162.409}, {23.297, 157.033}, {6.853, 90.448}, {-6.675, 35.271}, {-9.574, 29.059}, {-22.964, 19.948}, {-24.207, 3.521}, {-15.049, -11.229}, {-15.049, -11.229}, {-12.549, -11.229}}, smooth = Smooth.Bezier), Ellipse(visible = true, origin = {0, -90}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{27.5, 164.188}, {52.5, 189.188}}), Polygon(visible = true, origin = {17.312, -2.455}, lineColor = {128, 128, 128}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, points = {{-27.312, -59.815}, {-21.575, -57.545}, {-14.064, -57.545}, {-5.944, -60.627}, {0.146, -66.92}, {0.146, -66.92}, {-3.37, -60.294}, {-4.06, -53.116}, {24.507, 64.424}, {26.537, 69.702}, {26.537, 69.702}, {21.868, 69.702}, {16.923, 70.433}, {13.057, 71.813}, {9.078, 74.371}, {9.078, 74.371}, {9.078, 70.108}, {-19.935, -44.005}, {-22.558, -55.739}, {-27.312, -59.815}}, smooth = Smooth.Bezier), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -150}, {150, -110}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{20, 118}, {58, 140}}, textString = "t"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-144, 118}, {-106, 140}}, textString = "f"), Polygon(visible = true, origin = {0, 10}, rotation = 60, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{53.392, -68.478}, {43.392, -38.478}, {63.392, -38.478}}), Polygon(visible = true, origin = {0, 10}, rotation = -60, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{-53.392, -68.478}, {-63.392, -38.478}, {-43.392, -38.478}})}), Documentation(info = "<html>
<p>
The <b>3</b> signals of the <b>force</b> connector and the
<b>3</b> signals of the <b>torque</b> connector
are interpreted
as the x-, y- and z-coordinates of a <b>force</b> and of a
<b>torque</b> acting at the frame
connector to which frame_b of this component is attached.
Via parameter <b>resolveInFrame</b> it is defined, in which frame these
coordinates shall be resolved:
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>Types.ResolveInFrameAB.</b></th><th><b>Meaning</b></th></tr>
<tr><td valign=\"top\">world</td>
    <td valign=\"top\">Resolve input force/torque in world frame</td></tr>

<tr><td valign=\"top\">frame_a</td>
    <td valign=\"top\">Resolve input force/torque in frame_a</td></tr>

<tr><td valign=\"top\">frame_b</td>
    <td valign=\"top\">Resolve input force/torque in frame_b (= default)</td></tr>

<tr><td valign=\"top\">frame_resolve</td>
    <td valign=\"top\">Resolve input force/torque in frame_resolve (frame_resolve must be connected)</td></tr>
</table>

<p>
If resolveInFrame = ResolveInFrameAB.frame_resolve, the force and torque coordinates
are with respect to the frame, that is connected to <b>frame_resolve</b>.
</p>

<p>
If force={100,0,0}, and for all parameters the default setting is used,
then the interpretation is that a force of 100 N is acting along the positive
x-axis of frame_b.
</p>

<p>
Note, a force and torque acts on frame_a in such a way that
the force and torque balance between frame_a and frame_b is fulfilled.
</p>

<p>
An example how to use this model is given in the
following figure:
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/ForceAndTorque1.png\">
</p>

<p>
This leads to the following animation (the yellow cylinder
characterizes the line between frame_a and frame_b of the
ForceAndTorque component, i.e., the force and torque acts with
negative sign
also on the opposite side of this cylinder, but for
clarity this is not shown in the animation):
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/ForceAndTorque2.png\">
</p>

</html>"));
end ForceAndTorque;
