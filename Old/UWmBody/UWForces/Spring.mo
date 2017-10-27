within UWmBody.UWForces;

model Spring "Linear translational spring with optional mass"
  import UWmBody.UWTypes;
  extends UWInterfaces.PartialTwoFrames;
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter Boolean showMass = true "= true, if point mass shall be visualized as sphere if animation=true and m>0";
  parameter SI.TranslationalSpringConstant c(final min = 0) "Spring constant";
  parameter SI.Length s_unstretched = 0 "Unstretched spring length";
  parameter SI.Mass m(min = 0) = 0 "Spring mass located on the connection line between the origin of frame_a and the origin of frame_b";
  parameter Real lengthFraction(min = 0, max = 1) = 0.5 "Location of spring mass with respect to frame_a as a fraction of the distance from frame_a to frame_b (=0: at frame_a; =1: at frame_b)";
  input SI.Distance width = world.defaultForceWidth "Width of spring" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input SI.Distance coilWidth = width / 10 "Width of spring coil" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter Integer numberOfWindings = 5 "Number of spring windings" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.Color color = UWmBody.UWTypes.Defaults.SpringColor "Color of spring" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input Modelica.SIunits.Diameter massDiameter = max(0, (width - 2 * coilWidth) * 0.9) "Diameter of mass point sphere" annotation(Dialog(tab = "Animation", group = "if animation = true and showMass = true", enable = animation and showMass));
  input UWTypes.Color massColor = UWmBody.UWTypes.Defaults.BodyColor "Color of mass point" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true and showMass = true", enable = animation and showMass));
  parameter Boolean fixedRotationAtFrame_a = false "=true, if rotation frame_a.R is fixed (to directly connect line forces)" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Advanced", group = "If enabled, can give wrong results, see MultiBody.UsersGuide.Tutorial.ConnectionOfLineForces"));
  parameter Boolean fixedRotationAtFrame_b = false "=true, if rotation frame_b.R is fixed (to directly connect line forces)" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Advanced", group = "If enabled, can give wrong results, see MultiBody.UsersGuide.Tutorial.ConnectionOfLineForces"));
  UWForces.LineForceWithMass lineForce(animateLine = animation, animateMass = showMass, m = m, lengthFraction = lengthFraction, lineShapeType = "spring", lineShapeHeight = coilWidth * 2, lineShapeWidth = width, lineShapeExtra = numberOfWindings, lineShapeColor = color, specularCoefficient = specularCoefficient, massDiameter = massDiameter, massColor = massColor, fixedRotationAtFrame_a = fixedRotationAtFrame_a, fixedRotationAtFrame_b = fixedRotationAtFrame_b) annotation(Placement(transformation(extent = {{-20, -20}, {20, 20}})));
  Modelica.Mechanics.Translational.Components.Spring spring(s_rel0 = s_unstretched, c = c) annotation(Placement(transformation(extent = {{-8, 40}, {12, 60}})));
equation
  connect(lineForce.frame_a, frame_a) annotation(Line(points = {{-20, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(lineForce.frame_b, frame_b) annotation(Line(points = {{20, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(spring.flange_b, lineForce.flange_b) annotation(Line(points = {{12, 50}, {12, 20}}, color = {0, 191, 0}));
  connect(spring.flange_a, lineForce.flange_a) annotation(Line(points = {{-8, 50}, {-12, 50}, {-12, 20}}, color = {0, 191, 0}));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 56}, {150, 96}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -80}, {150, -50}}, textString = "c=%c"), Ellipse(visible = fixedRotationAtFrame_a, lineColor = {255, 0, 0}, extent = {{-130, -30}, {-70, 30}}), Text(visible = fixedRotationAtFrame_a, textColor = {255, 0, 0}, extent = {{-140, 30}, {-62, 50}}, textString = "R=0"), Ellipse(visible = fixedRotationAtFrame_b, lineColor = {255, 0, 0}, extent = {{70, -30}, {130, 30}}), Text(visible = fixedRotationAtFrame_b, textColor = {255, 0, 0}, extent = {{62, 30}, {140, 50}}, textString = "R=0"), Polygon(visible = true, origin = {6.698, -0.12}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, points = {{-106.698, 4.12}, {-67.698, 4.12}, {-54.698, -23.88}, {-26.698, 34.12}, {-18.698, 34.12}, {9.302, -23.88}, {37.302, 34.12}, {45.302, 34.12}, {59.302, 4.12}, {93.302, 4.12}, {93.302, -3.88}, {54.302, -3.88}, {41.302, 24.12}, {13.302, -33.88}, {5.302, -33.88}, {-22.698, 24.12}, {-50.698, -33.88}, {-58.698, -33.88}, {-72.698, -3.88}, {-106.698, -3.88}}), Polygon(visible = true, origin = {-56.75, -14.5}, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{-4.25, 18.5}, {8.75, -9.5}, {4.75, -19.5}, {-9.25, 10.5}}), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{-16, 24}, {-12, 34}, {16, -24}, {12, -34}}), Polygon(visible = true, origin = {56.75, 14.5}, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{-8.75, 9.5}, {-4.75, 19.5}, {9.25, -10.5}, {4.25, -18.5}}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, extent = {{-10, -10}, {10, 10}}), Polygon(visible = true, origin = {-81.75, 0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, points = {{-18.25, 4}, {20.75, 4}, {15.75, -4}, {-18.25, -4}}), Polygon(visible = true, origin = {81.75, 0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, points = {{-15.75, 4}, {18.25, 4}, {18.25, -4}, {-20.75, -4}})}), Documentation(info = "<html>
<p>
<b>Linear spring</b> acting as line force between frame_a and frame_b.
A <b>force f</b> is exerted on the origin of frame_b and with opposite sign
on the origin of frame_a along the line from the origin of frame_a to the origin
of frame_b according to the equation:
</p>
<pre>
   f = c*(s - s_unstretched);
</pre>
<p>
where \"c\" and \"s_unstretched\" are parameters and \"s\" is the
distance between the origin of frame_a and the origin of frame_b.
</p>
<p>
Optionally, the mass of the spring is taken into account by a
point mass located on the line between frame_a and frame_b
(default: middle of the line). If the spring mass is zero, the
additional equations to handle the mass are removed.
</p>
<p>
In the following figure a typical animation of the
spring is shown. The blue sphere in the middle of the
spring characterizes the location of the point mass.
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/SpringWithMass.png\"
ALT=\"model Examples.Elementary.SpringWithMass\">
</p>

</html>"));
end Spring;
