within UWmBody.UWExamples.Loops;

model Engine1b "Model of one cylinder engine with gas force and preparation for assembly joint JointRRP"
  extends Modelica.Icons.Example;
  extends Utilities.Engine1bBase(Inertia(w(start = 0)));
  UWJoints.RevolutePlanarLoopConstraint B2(n = {1, 0, 0}, cylinderLength = 0.02, cylinderDiameter = 0.05) annotation(Placement(transformation(extent = {{40, 20}, {60, 40}})));
  UWmBody.UWJoints.Revolute B1(n = {1, 0, 0}, cylinderLength = 0.02, cylinderDiameter = 0.05) annotation(Placement(transformation(extent = {{40, -20}, {60, 0}})));
  UWmBody.UWJoints.Prismatic Cylinder(useAxisFlange = true, boxWidth = 0.02, n = {0, -1, 0}) annotation(Placement(transformation(origin = {50, 95}, extent = {{-10, -10}, {10, 10}}, rotation = 270), visible = true));
  UWParts.FixedTranslation Rod1(r = {0, 0.2, 0}, animation = false) annotation(Placement(transformation(origin = {70, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  UWParts.FixedTranslation Rod3(r = {0, -0.1, 0}, animation = false) annotation(Placement(transformation(origin = {50, 60}, extent = {{10, -10}, {-10, 10}}, rotation = 90), visible = true));
equation
  connect(B1.frame_b, Rod1.frame_a) annotation(Line(points = {{60, -10}, {70, -10}, {70, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(Rod1.frame_b, B2.frame_b) annotation(Line(points = {{70, 20}, {70, 30}, {60, 30}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(Cylinder.frame_b, Rod3.frame_a) annotation(Line(points = {{0, 7.5}, {0, -7.5}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {50, 77.5}));
  connect(B2.frame_a, Rod3.frame_b) annotation(Line(points = {{40, 30}, {30, 30}, {30, 45}, {50, 45}, {50, 50}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(cylPosition.frame_b, Cylinder.frame_a) annotation(Line(points = {{-46.667, 1.667}, {23.333, 1.667}, {23.333, -3.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {26.667, 108.333}));
  connect(gasForce.flange_a, Cylinder.support) annotation(Line(points = {{120, 105}, {120, 110}, {70, 110}, {70, 99}, {56, 99}}, color = {0, 127, 0}, visible = true));
  connect(Cylinder.axis, gasForce.flange_b) annotation(Line(points = {{56, 87}, {70, 87}, {70, 80}, {120, 80}, {120, 85}}, color = {0, 127, 0}, visible = true));
  connect(Piston.frame_a, Rod3.frame_a) annotation(Line(points = {{120, 60}, {120, 75}, {50, 75}, {50, 70}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(B1.frame_b, Rod2.frame_a) annotation(Line(points = {{-40, -3.333}, {20, -3.333}, {20, 6.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {100, -6.667}));
  connect(Mid.frame_b, B1.frame_a) annotation(Line(points = {{50, -45}, {60, -45}, {60, -30}, {30, -30}, {30, -10}, {40, -10}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  annotation(experiment(StopTime = 0.5), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -120}, {150, 120}}, initialScale = 0.1, grid = {5, 5}), graphics = {Rectangle(visible = true, origin = {17.706, -2}, lineColor = {192, 192, 192}, fillColor = {255, 255, 255}, lineThickness = 1, extent = {{2.294, -23}, {67.294, 117}}, radius = 5), Text(visible = true, origin = {-13.125, 1}, textColor = {128, 128, 128}, extent = {{73.125, -36}, {163.125, -28}}, textString = "jointRRP in model", textStyle = {TextStyle.Bold, TextStyle.Italic}), Text(visible = true, origin = {-19.2, -1}, textColor = {128, 128, 128}, extent = {{79.2, -42}, {169.2, -34}}, textString = "Loops.Engine1b_analytic", textStyle = {TextStyle.Bold, TextStyle.Italic})}), Documentation(info = "<html>
<p>
This is a model of the mechanical part of one cylinder of an engine.
It is similar to
<a href=\"modelica://Modelica.Mechanics.MultiBody.Examples.Loops.Engine1a#diagram\">Loops.Engine1a</a>.
The difference is that a simple
model for the gas force in the cylinder is added and that the
model is restructured in such a way, that the central part of
the planar kinematic loop can be easily replaced by the
assembly joint \"Modelica.Mechanics.MultiBody.Joints.Assemblies.<b>JointRRP</b>\".
This exchange of the kinematic loop is shown in
<a href=\"modelica://Modelica.Mechanics.MultiBody.Examples.Loops.Engine1b_analytic#diagram\">Loops.Engine1b_analytic</a>.
The advantage of using JointRRP is, that the
non-linear algebraic equation of this loop is solved analytically, and
not numerically as in this model (Engine1b).
</p>
<p>
An animation of this example is shown in the figure below.
</p>

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Loops/Engine.png\" ALT=\"model Examples.Loops.Engine\">
</html>"));
end Engine1b;
