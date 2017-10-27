within UWBody.Examples.Loops;

model Fourbar_analytic "One kinematic loop with four bars (with JointSSP joint; analytic solution of non-linear algebraic loop)"
  extends Modelica.Icons.Example;
  output Modelica.SIunits.Angle j1_phi "angle of revolute joint j1";
  output SI.Position j2_s "distance of prismatic joint j2";
  output SI.AngularVelocity j1_w "axis speed of revolute joint j1";
  output Modelica.SIunits.Velocity j2_v "axis velocity of prismatic joint j2";
  inner UWBody.World world(animateGravity = false) annotation(Placement(transformation(extent = {{-80, -60}, {-60, -40}}, origin = {0, 0}, rotation = 0), visible = true));
  UWBody.Joints.Revolute j1(useAxisFlange = true, n = {1, 0, 0}, stateSelect = StateSelect.always, phi(fixed = true), w(displayUnit = "deg/s", start = 5.235987755982989, fixed = true)) annotation(Placement(transformation(extent = {{-54, -40}, {-34, -20}}, origin = {4, 0}, rotation = 0), visible = true));
  UWBody.Parts.BodyCylinder b1(r = {0, 0.5, 0.1}, diameter = 0.05) annotation(Placement(transformation(origin = {-20, -0}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true));
  UWBody.Parts.FixedTranslation b3(r = {1.2, 0, 0}, animation = false) annotation(Placement(transformation(extent = {{-32, -60}, {-12, -40}}, origin = {10, 0}, rotation = 0), visible = true));
  UWBody.Joints.Assemblies.JointSSP jointSSP(rod1Length = sqrt({-1, 0.3, 0.1} * {-1, 0.3, 0.1}), n_b = {1, 0, 0}, s_offset = -0.2, rRod2_ib = {0, 0.2, 0}, rod1Color = {0, 128, 255}, rod2Color = {0, 128, 255}, checkTotalPower = true) annotation(Placement(transformation(extent = {{-20, 0}, {20, 40}}, origin = {10, 10}, rotation = 0), visible = true));
  UWBody.Parts.BodyCylinder b2(r = {0, 0.2, 0}, diameter = 0.05, animation = false) annotation(Placement(transformation(origin = {60, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90), visible = true));
equation
  j1_phi = j1.phi;
  j2_s = jointSSP.prismatic.distance;
  j1_w = j1.w;
  j2_v = der(jointSSP.prismatic.distance);
  connect(j1.frame_b, b1.frame_a) annotation(Line(points = {{-6.667, -6.667}, {3.333, -6.667}, {3.333, 13.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-23.333, -23.333}));
  connect(j1.frame_a, world.frame_b) annotation(Line(points = {{-60, -30}, {-70, -30}, {-70, -50}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, 0}));
  connect(b3.frame_a, world.frame_b) annotation(Line(points = {{19, 0}, {-19, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-41, -50}));
  connect(b1.frame_b, jointSSP.frame_a) annotation(Line(points = {{-3.333, -13.333}, {-3.333, 6.667}, {6.667, 6.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-16.667, 23.333}));
  connect(b3.frame_b, jointSSP.frame_b) annotation(Line(points = {{-12, -50}, {30, -50}, {30, 30}, {20, 30}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, 0}));
  connect(b2.frame_a, jointSSP.frame_ib) annotation(Line(points = {{50, 40}, {50, 60}, {16, 60}, {16, 50}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, 0}));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Joints j1 and j2", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = j1_phi, legend = "Angle of revolute joint j1")}), SubPlot(curves = {Curve(x = time, y = j2_s, legend = "Distance of prismatic joint j2 (jointSSP.prismatic)")}), SubPlot(curves = {Curve(x = time, y = j1_w, legend = "Angular velocity of revolute joint j1"), Curve(x = time, y = j2_v, legend = "Velocity of prismatic joint j2 (jointSSP.prismatic)")})})})), experiment(StopTime = 5), Documentation(info = "<html>
<p>
This is a third version of the \"four-bar\" mechanism, see figure:
</p>

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Loops/Fourbar_analytic.png\" ALT=\"model Examples.Loops.Fourbar_analytic\">

<p>
In this case
the three revolute joints on the left top-side and the two revolute
joints on the right top side have been replaced by the assembly joint
<b>Joints.Assemblies.JointSSP</b>
which consists of two spherical joints and one prismatic joint.
Since JointSSP solves the non-linear constraint equation internally
analytically, no non-linear equation appears any more and a Modelica
translator can transform the system into state space
form without solving a system of equations. For more details, see
<a href=\"modelica://UWBody.UsersGuide.Tutorial.LoopStructures.AnalyticLoopHandling\">
MultiBody.UsersGuide.Tutorial.LoopStructures.AnalyticLoopHandling</a>.
</p>
</html>"));
end Fourbar_analytic;
