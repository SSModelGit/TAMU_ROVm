within UWBody.Examples.Loops;

model Fourbar1 "One kinematic loop with four bars (with only revolute joints; 5 non-linear equations)"
  extends Modelica.Icons.Example;
  output Modelica.SIunits.Angle j1_phi "angle of revolute joint j1";
  output SI.Position j2_s "distance of prismatic joint j2";
  output SI.AngularVelocity j1_w "axis speed of revolute joint j1";
  output Modelica.SIunits.Velocity j2_v "axis velocity of prismatic joint j2";
  inner UWBody.World world annotation(Placement(transformation(extent = {{-100, -80}, {-80, -60}}, origin = {10, -0}, rotation = 0), visible = true));
  UWBody.Joints.Revolute j1(n = {1, 0, 0}, stateSelect = StateSelect.always, phi(fixed = true), w(displayUnit = "deg/s", start = 5.235987755982989, fixed = true)) annotation(Placement(transformation(extent = {{-54, -40}, {-34, -20}}, origin = {4, -0}, rotation = 0), visible = true));
  UWBody.Joints.Prismatic j2(n = {1, 0, 0}, s(start = -0.2), boxWidth = 0.05) annotation(Placement(transformation(extent = {{10, -80}, {30, -60}}, origin = {-10, -0}, rotation = 0), visible = true));
  UWBody.Parts.BodyCylinder b1(r = {0, 0.5, 0.1}, diameter = 0.05) annotation(Placement(transformation(origin = {-20, -0}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true));
  UWBody.Parts.BodyCylinder b2(r = {0, 0.2, 0}, diameter = 0.05) annotation(Placement(transformation(origin = {40, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true));
  UWBody.Parts.BodyCylinder b3(r = {-1, 0.3, 0.1}, diameter = 0.05) annotation(Placement(transformation(extent = {{38, 20}, {18, 40}}, origin = {12, -0}, rotation = 0), visible = true));
  UWBody.Joints.Revolute rev(n = {0, 1, 0}) annotation(Placement(transformation(origin = {40, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true));
  UWBody.Joints.Revolute rev1 annotation(Placement(transformation(extent = {{60, 0}, {80, 20}}, origin = {-0, -0}, rotation = 0), visible = true));
  UWBody.Joints.Revolute j3(n = {1, 0, 0}) annotation(Placement(transformation(extent = {{-60, 40}, {-40, 60}}, origin = {10, 10}, rotation = 0), visible = true));
  UWBody.Joints.Revolute j4(n = {0, 1, 0}) annotation(Placement(transformation(extent = {{-32, 60}, {-12, 80}}, origin = {22, -0}, rotation = 0), visible = true));
  UWBody.Joints.Revolute j5(n = {0, 0, 1}) annotation(Placement(transformation(extent = {{0, 70}, {20, 90}}, origin = {30, -0}, rotation = 0), visible = true));
  UWBody.Parts.FixedTranslation b0(animation = false, r = {1.2, 0, 0}) annotation(Placement(transformation(extent = {{-40, -80}, {-20, -60}}, origin = {0, -0}, rotation = 0), visible = true));
equation
  connect(j2.frame_b, b2.frame_a) annotation(Line(points = {{-13.333, -3.333}, {6.667, -3.333}, {6.667, 6.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {33.333, -66.667}));
  connect(j1.frame_b, b1.frame_a) annotation(Line(points = {{-6.667, -6.667}, {3.333, -6.667}, {3.333, 13.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-23.333, -23.333}));
  connect(rev.frame_a, b2.frame_b) annotation(Line(points = {{0, 5}, {0, -5}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {40, -35}));
  connect(rev.frame_b, rev1.frame_a) annotation(Line(points = {{-6.667, -13.333}, {-6.667, 6.667}, {13.333, 6.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {46.667, 3.333}));
  connect(rev1.frame_b, b3.frame_a) annotation(Line(points = {{70, 10}, {80, 10}, {80, 30}, {40, 30}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, -0}));
  connect(world.frame_b, j1.frame_a) annotation(Line(points = {{-80, -70}, {-70, -70}, {-70, -30}, {-60, -30}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, -0}));
  connect(b1.frame_b, j3.frame_a) annotation(Line(points = {{-30, 10}, {-30, 30}, {-70, 30}, {-70, 60}, {-60, 60}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, -0}));
  connect(j3.frame_b, j4.frame_a) annotation(Line(points = {{-40, 60}, {-30, 60}, {-30, 70}, {-20, 70}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, -0}));
  connect(j4.frame_b, j5.frame_a) annotation(Line(points = {{0, 70}, {0, 70}, {10, 70}, {10, 80}, {20, 80}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, -0}));
  connect(j5.frame_b, b3.frame_b) annotation(Line(points = {{40, 80}, {50, 80}, {50, 50}, {10, 50}, {10, 30}, {20, 30}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, -0}));
  connect(b0.frame_a, world.frame_b) annotation(Line(points = {{15, 0}, {-15, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-55, -70}));
  connect(b0.frame_b, j2.frame_a) annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-10, -70}));
  j1_phi = j1.phi;
  j2_s = j2.s;
  j1_w = j1.w;
  j2_v = j2.v;
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "j1 and j2", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = j1_phi, legend = "Angle of revolute joint j1")}, range = Range(xmin = 0, xmax = 5, ymin = auto, ymax = auto)), SubPlot(curves = {Curve(x = time, y = j2_s, legend = "Distance of prismatic joint j2")}), SubPlot(curves = {Curve(x = time, y = j1_w, legend = "Angular velocity of revolute joint j1"), Curve(x = time, y = j2_v, legend = "Velocity of revolute joint j2")}, range = Range(xmin = 0, xmax = 5, ymin = auto, ymax = auto))})})), experiment(StopTime = 5), Documentation(info = "<html>
<p>
This is a simple kinematic loop consisting of 6 revolute joints, 1 prismatic joint
and 4 bars that is often used as basic constructing unit in mechanisms.
This example demonstrates that usually no particular knowledge
of the user is needed to handle kinematic loops.
Just connect the joints and bodies together according
to the real system. In particular <b>no</b> cut-joints or a spanning tree has
to be determined. In this case, the initial condition of the angular velocity
of revolute joint j1 is set to 300 deg/s in order to drive this loop.
</p>

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Loops/Fourbar1.png\" ALT=\"model Examples.Loops.Fourbar1\">
</html>"));
end Fourbar1;
