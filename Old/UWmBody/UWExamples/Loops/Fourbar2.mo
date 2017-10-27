within UWmBody.UWExamples.Loops;

model Fourbar2 "One kinematic loop with four bars (with UniversalSpherical joint; 1 non-linear equation)"
  extends Modelica.Icons.Example;
  output Modelica.SIunits.Angle j1_phi "angle of revolute joint j1";
  output SI.Position j2_s "distance of prismatic joint j2";
  output SI.AngularVelocity j1_w "axis speed of revolute joint j1";
  output Modelica.SIunits.Velocity j2_v "axis velocity of prismatic joint j2";
  inner UWmBody.UWWorld world annotation(Placement(transformation(extent = {{-80, -80}, {-60, -60}}, origin = {0, 0}, rotation = 0), visible = true));
  UWmBody.UWJoints.Revolute j1(useAxisFlange = true, n = {1, 0, 0}, stateSelect = StateSelect.always, phi(fixed = true), w(displayUnit = "deg/s", start = 5.235987755982989, fixed = true)) annotation(Placement(transformation(extent = {{-54, -40}, {-34, -20}}, origin = {4, 0}, rotation = 0), visible = true));
  UWmBody.UWJoints.Prismatic j2(n = {1, 0, 0}, boxWidth = 0.05, s(fixed = true, start = -0.2)) annotation(Placement(transformation(extent = {{12, -80}, {32, -60}}, origin = {-2, 0}, rotation = 0), visible = true));
  UWmBody.UWParts.BodyCylinder b1(r = {0, 0.5, 0.1}, diameter = 0.05) annotation(Placement(transformation(origin = {-20, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true));
  UWmBody.UWParts.BodyCylinder b2(r = {0, 0.2, 0}, diameter = 0.05) annotation(Placement(transformation(origin = {50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true));
  UWmBody.UWJoints.UniversalSpherical universalSpherical(n1_a = {0, 1, 0}, computeRodLength = true, rRod_ia = {-1, 0.3, 0.1}) annotation(Placement(transformation(extent = {{0, 18}, {-20, 38}}, origin = {20, 2}, rotation = 0), visible = true));
  UWmBody.UWParts.FixedTranslation b3(r = {1.2, 0, 0}, animation = false) annotation(Placement(transformation(extent = {{-32, -80}, {-12, -60}}, origin = {2, 0}, rotation = 0), visible = true));
  UWmBody.UWVisualizers.FixedFrame fixedFrame(color_x = {0, 0, 255}) annotation(Placement(transformation(origin = {14.003, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true));
equation
  j1_phi = j1.phi;
  j2_s = j2.s;
  j1_w = j1.w;
  j2_v = j2.v;
  connect(j2.frame_b, b2.frame_a) annotation(Line(points = {{-13.333, -10}, {6.667, -10}, {6.667, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {43.333, -60}));
  connect(j1.frame_b, b1.frame_a) annotation(Line(points = {{-6.667, -6.667}, {3.333, -6.667}, {3.333, 13.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-23.333, -23.333}));
  connect(j1.frame_a, world.frame_b) annotation(Line(points = {{-60, -30}, {-70, -30}, {-70, -70}, {-70, -70}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, 0}));
  connect(b1.frame_b, universalSpherical.frame_b) annotation(Line(points = {{-6.667, -13.333}, {-6.667, 6.667}, {13.333, 6.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-13.333, 23.333}));
  connect(universalSpherical.frame_a, b2.frame_b) annotation(Line(points = {{-20, 16.667}, {10, 16.667}, {10, -33.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {40, 13.333}));
  connect(b3.frame_a, world.frame_b) annotation(Line(points = {{15, 0}, {-15, -0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-45, -70}));
  connect(b3.frame_b, j2.frame_a) annotation(Line(points = {{-10, -0}, {10, -0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -70}));
  connect(fixedFrame.frame_a, universalSpherical.frame_ia) annotation(Line(points = {{4.003, 60}, {4, 40}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, 0}));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Joints j1 and j2", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = j1_phi, legend = "Angle of revolute joint j1")}), SubPlot(curves = {Curve(x = time, y = j2_s, legend = "Distance of prismatic joint j2")}), SubPlot(curves = {Curve(x = time, y = j1_w, legend = "Angular velocity of revolute joint j1"), Curve(x = time, y = j2_v, legend = "Velocity of prismatic joint j2")})})})), experiment(StopTime = 5), Documentation(info = "<html>
<p>
This is a second version of the \"four-bar\" mechanism, see figure:
</p>

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Loops/Fourbar2.png\" ALT=\"model Examples.Loops.Fourbar2\">

<p>
In this case
the three revolute joints on the left top-side and the two revolute
joints on the right top side have been replaced by the joint <b>UniversalSpherical</b>
that is a rod connecting a spherical and a universal joint. This joint is defined
by <b>1 constraint</b> stating that the distance between the two spherical joints is
constant. Using this joint in a kinematic loop reduces the sizes of
non-linear algebraic equations. For this loop, only one non-linear
algebraic system of equations of order 1 remains.
</p>
<p>
At the UniversalSpherical joint an additional frame_ia fixed to the rod
is present where components can be attached to the connecting rod. In this
example just a coordinate system is attached to visualize frame_ia (coordinate
system on the right in blue color).
</p>
<p>
Another feature is that the length of the connecting rod can be
automatically calculated during <b>initialization</b>. In order to do this,
another initialization condition has to be given. In this example, the
initial value of the distance of the prismatic joint j2 has been fixed
(via the \"Initialization\" menu) and the rod length of joint
\"UniversalSpherical\" is computed during initialization since parameter
<b>computeLength</b> = <b>true</b> is set in the joint parameter
menu. The main advantage is that during initialization no non-linear
system of equation is solved and therefore initialization always works.
To be precise, the following trivial non-linear equation is actually solved
for rodLength:
</p>
<pre>
   rodLength*rodLength = f(angle of revolute joint, distance of prismatic joint)
</pre>
</html>"));
end Fourbar2;
