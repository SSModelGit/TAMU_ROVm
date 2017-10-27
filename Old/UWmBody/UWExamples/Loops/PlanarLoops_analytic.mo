within UWmBody.UWExamples.Loops;

model PlanarLoops_analytic "Mechanism with three planar kinematic loops and one degree-of-freedom with analytic loop handling (with JointRRR joints)"
  extends Modelica.Icons.Example;
  parameter SI.Length rh[3] = {0.5, 0, 0} "Position vector from 'lower left' revolute to 'lower right' revolute joint for all the 3 loops";
  parameter SI.Length rv[3] = {0, 0.5, 0} "Position vector from 'lower left' revolute to 'upper left' revolute joint in the first loop";
  parameter SI.Length r1b[3] = {0.1, 0.5, 0} "Position vector from 'lower right' revolute to 'upper right' revolute joint in the first loop";
  final parameter SI.Length r1a[3] = r1b + rh - rv "Position vector from 'upper left' revolute to 'upper right' revolute joint in the first loop";
  parameter SI.Length r2b[3] = {0.1, 0.6, 0} "Position vector from 'lower right' revolute to 'upper right' revolute joint in the second loop";
  final parameter SI.Length r2a[3] = r2b + rh - r1b "Position vector from 'upper left' revolute to 'upper right' revolute joint in the second loop";
  parameter SI.Length r3b[3] = {0, 0.55, 0} "Position vector from 'lower right' revolute to 'upper right' revolute joint in the third loop";
  final parameter SI.Length r3a[3] = r3b + rh - r2b "Position vector from 'upper left' revolute to 'upper right' revolute joint in the third loop";
  inner UWmBody.UWWorld world annotation(Placement(transformation(extent = {{-94, -90}, {-74, -70}}, origin = {-1, 10}, rotation = 0), visible = true));
  UWmBody.UWJoints.Assemblies.JointRRR jointRRR1(rRod1_ia = r1a, rRod2_ib = r1b, checkTotalPower = true) annotation(Placement(transformation(origin = {-20, 10}, extent = {{-20, -20}, {20, 20}}, rotation = 270), visible = true));
  UWmBody.UWJoints.Revolute rev(useAxisFlange = true, w(fixed = true)) annotation(Placement(transformation(origin = {-55, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true));
  UWmBody.UWParts.FixedTranslation rod1(r = rv) annotation(Placement(transformation(origin = {-55, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true));
  UWmBody.UWParts.FixedTranslation rod2(r = rh) annotation(Placement(transformation(extent = {{-50, -60}, {-30, -40}}, origin = {2.5, 10}, rotation = 0), visible = true));
  UWmBody.UWParts.Body body1(m = 1, cylinderColor = {155, 155, 155}, r_CM = jointRRR1.rRod1_ia / 2) annotation(Placement(transformation(origin = {5, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true));
  Modelica.Mechanics.Rotational.Sources.Position position(useSupport = true) annotation(Placement(transformation(extent = {{-90, -20}, {-70, 0}}, origin = {-0, 10}, rotation = 0), visible = true));
  Modelica.Blocks.Sources.Sine sine(amplitude = 0.7, freqHz = 1) annotation(Placement(transformation(origin = {-100, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 270), visible = true));
  UWmBody.UWJoints.Assemblies.JointRRR jointRRR2(rRod1_ia = r2a, rRod2_ib = r2b, checkTotalPower = true) annotation(Placement(transformation(origin = {30, 10}, extent = {{-20, -20}, {20, 20}}, rotation = 270), visible = true));
  UWmBody.UWParts.FixedTranslation rod3(r = rh) annotation(Placement(transformation(extent = {{0, -60}, {20, -40}}, origin = {-5, 10}, rotation = 0), visible = true));
  UWmBody.UWParts.Body body2(m = 1, cylinderColor = {155, 155, 155}, r_CM = jointRRR2.rRod1_ia / 2) annotation(Placement(transformation(origin = {55, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true));
  UWmBody.UWJoints.Assemblies.JointRRR jointRRR3(rRod1_ia = r3a, rRod2_ib = r3b, checkTotalPower = true) annotation(Placement(transformation(origin = {80, 10}, extent = {{-20, -20}, {20, 20}}, rotation = 270), visible = true));
  UWmBody.UWParts.FixedTranslation rod4(r = rh) annotation(Placement(transformation(extent = {{40, -60}, {60, -40}}, origin = {5, 10}, rotation = 0), visible = true));
  UWmBody.UWParts.Body body3(m = 1, cylinderColor = {155, 155, 155}, r_CM = jointRRR3.rRod1_ia / 2) annotation(Placement(transformation(origin = {105, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true));
  UWParts.Mounting1D mounting1D annotation(Placement(transformation(extent = {{-100, -50}, {-80, -30}}, origin = {-0, 10}, rotation = 0), visible = true));
equation
  connect(world.frame_b, rev.frame_a) annotation(Line(points = {{-13.333, -20}, {6.667, -20}, {6.667, 40}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-61.667, -50}));
  connect(rod1.frame_a, rev.frame_b) annotation(Line(points = {{0, 10}, {0, -10}}, thickness = 0.5, visible = true, color = {95, 95, 95}, origin = {-55, 20}));
  connect(rod1.frame_b, jointRRR1.frame_a) annotation(Line(points = {{-55, 40}, {-55, 50}, {-20, 50}, {-20, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-0, 10}));
  connect(rod2.frame_a, world.frame_b) annotation(Line(points = {{-47.5, -50}, {-55, -50}, {-55, -80}, {-75, -80}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-0, 10}));
  connect(rod2.frame_b, jointRRR1.frame_b) annotation(Line(points = {{-5, -10}, {2.5, -10}, {2.5, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-22.5, -30}));
  connect(jointRRR1.frame_ia, body1.frame_a) annotation(Line(points = {{-3.333, -11.333}, {1.667, -11.333}, {1.667, 22.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {3.333, 37.333}));
  connect(rod3.frame_a, rod2.frame_b) annotation(Line(points = {{11.25, -0}, {-11.25, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-16.25, -40}));
  connect(rod3.frame_b, jointRRR2.frame_b) annotation(Line(points = {{-10, -10}, {5, -10}, {5, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {25, -30}));
  connect(jointRRR2.frame_ia, body2.frame_a) annotation(Line(points = {{-3.333, -11.333}, {1.667, -11.333}, {1.667, 22.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {53.333, 37.333}));
  connect(jointRRR1.frame_im, jointRRR2.frame_a) annotation(Line(points = {{0, 0}, {12, 0}, {12, 25}, {30, 25}, {30, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-0, 10}));
  connect(rod3.frame_b, rod4.frame_a) annotation(Line(points = {{-15, 0}, {15, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {30, -40}));
  connect(rod4.frame_b, jointRRR3.frame_b) annotation(Line(points = {{-10, -10}, {5, -10}, {5, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {75, -30}));
  connect(jointRRR2.frame_im, jointRRR3.frame_a) annotation(Line(points = {{50, 0}, {60, 0}, {60, 25}, {80, 25}, {80, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-0, 10}));
  connect(jointRRR3.frame_ia, body3.frame_a) annotation(Line(points = {{-3.333, -11.333}, {1.667, -11.333}, {1.667, 22.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {103.333, 37.333}));
  connect(sine.y, position.phi_ref) annotation(Line(points = {{-2.667, 19.333}, {-2.667, -9.667}, {5.333, -9.667}}, color = {1, 37, 163}, visible = true, origin = {-97.333, 9.667}));
  connect(mounting1D.flange_b, position.support) annotation(Line(points = {{-80, -40}, {-80, -20}}, visible = true, color = {64, 64, 64}, origin = {-0, 10}));
  connect(mounting1D.frame_a, world.frame_b) annotation(Line(points = {{-90, -50}, {-90, -55}, {-70, -55}, {-70, -80}, {-75, -80}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-0, 10}));
  connect(position.flange, rev.axis) annotation(Line(points = {{-2.5, 0}, {2.5, 0}}, visible = true, color = {64, 64, 64}, origin = {-67.5, 0}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})), experiment(StopTime = 5), Documentation(info = "<html>
<p>
It is demonstrated how the Modelica.Mechanics.MultiBody.Joints.Assemblies.JointRRR joint can be
used to solve the non-linear equations of coupled planar loops analytically.
In the mechanism below no non-linear equation occurs any more from the tool
view, since these equations are solved analytically in the JointRRR joints.
For more details, see
<a href=\"modelica://Modelica.Mechanics.MultiBody.UsersGuide.Tutorial.LoopStructures.AnalyticLoopHandling\">
MultiBody.UsersGuide.Tutorial.LoopStructures.AnalyticLoopHandling</a>.
</p>

<p>
In the following figure the parameter vectors of this example are visualized in the
animation view.
</p>

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Loops/PlanarLoops2.png\" ALT=\"model Examples.Loops.PlanarLoops2\">
</html>"));
end PlanarLoops_analytic;
