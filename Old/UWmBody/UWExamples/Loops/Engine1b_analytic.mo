within UWmBody.UWExamples.Loops;

model Engine1b_analytic "Model of one cylinder engine with gas force and analytic loop handling"
  extends Modelica.Icons.Example;
  extends Utilities.Engine1bBase(Inertia(w(start = 0)));
  UWJoints.Assemblies.JointRRP jointRRP(n_a = {1, 0, 0}, n_b = {0, -1, 0}, animation = false, rRod1_ia = {0, 0.2, 0}, rRod2_ib = {0, -0.1, 0}) annotation(Placement(transformation(origin = {30, 55}, extent = {{-15, 15}, {15, -15}}, rotation = 90), visible = true));
equation
  connect(Mid.frame_b, jointRRP.frame_a) annotation(Line(points = {{50, -45}, {60, -45}, {60, -30}, {30, -30}, {30, 40}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(jointRRP.frame_b, cylPosition.frame_b) annotation(Line(points = {{16.667, -26.667}, {16.667, 13.333}, {-33.333, 13.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {13.333, 96.667}));
  connect(jointRRP.axis, gasForce.flange_b) annotation(Line(points = {{42.828, 72}, {42.828, 82}, {120.828, 82}, {120.828, 87}}, color = {0, 127, 0}, visible = true, origin = {-0.828, -2}));
  connect(jointRRP.bearing, gasForce.flange_a) annotation(Line(points = {{36, 70}, {36, 110}, {120, 110}, {120, 105}}, color = {0, 127, 0}, visible = true));
  connect(jointRRP.frame_ib, Piston.frame_a) annotation(Line(points = {{-50, 2.333}, {25, 2.333}, {25, -4.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {95, 64.667}));
  connect(jointRRP.frame_ia, Rod2.frame_a) annotation(Line(points = {{45, 43}, {80, 43}, {80, -10}, {120, -10}, {120, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  annotation(experiment(StopTime = 0.5), Documentation(info = "<html>
<p>
This is the same model as
<a href=\"modelica://Modelica.Mechanics.MultiBody.Examples.Loops.Engine1b#diagram\">Loops.Engine1b</a>.
The only difference is that the central part of
the planar kinematic loop has been replaced by the
assembly joint \"Modelica.Mechanics.MultiBody.Joints.Assemblies.<b>JointRRP</b>\".
The advantage of using JointRRP is, that the
non-linear algebraic equation of this loop is solved analytically, and
not numerically as in
<a href=\"modelica://Modelica.Mechanics.MultiBody.Examples.Loops.Engine1b#diagram\">Loops.Engine1b</a>.
</p>
<p>
An animation of this example is shown in the figure below.
</p>

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Loops/Engine.png\" ALT=\"model Examples.Loops.Engine\">
</html>"));
end Engine1b_analytic;
