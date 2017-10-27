within UWmBody.UWExamples.Elementary;

model LineForceWithTwoMasses "Demonstrate line force with two point masses using a JointUPS and alternatively a LineForceWithTwoMasses component"
  extends Modelica.Icons.Example;
  parameter Modelica.SIunits.Mass m = 1 "Mass of point masses";
  SI.Force rod_f_diff[3] = rod1.frame_b.f - rod3.frame_b.f "Difference of cut-forces in rod1 and rod3";
  SI.Force body_f_diff[3] = bodyBox1.frame_b.f - bodyBox2.frame_b.f "Difference of cut-forces in bodyBox1 and bodyBox2";
  inner UWmBody.UWWorld world annotation(Placement(transformation(extent = {{-80, 60}, {-60, 80}}, origin = {0, -0}, rotation = 0), visible = true));
  UWmBody.UWJoints.Revolute revolute1(phi(fixed = true), w(fixed = true)) annotation(Placement(transformation(extent = {{-20, 60}, {0, 80}}, origin = {0, -0}, rotation = 0), visible = true));
  UWmBody.UWParts.BodyBox bodyBox1(r = {0.7, 0, 0}) annotation(Placement(transformation(extent = {{20, 60}, {40, 80}}, origin = {0, -0}, rotation = 0), visible = true));
  UWmBody.UWParts.FixedTranslation rod1(r = {0, -0.9, 0}, width = 0.01, animation = false) annotation(Placement(transformation(origin = {-40, 55}, extent = {{-10, -10}, {10, 10}}, rotation = 270), visible = true));
  UWmBody.UWJoints.Assemblies.JointUPS jointUPS(nAxis_ia = {0.7, 1.2, 0}, animation = true) annotation(Placement(transformation(extent = {{0, 50}, {20, 30}}, origin = {0, -0}, rotation = 0), visible = true));
  UWmBody.UWParts.Body body1(r_CM = 0.2 * jointUPS.eAxis_ia, cylinderDiameter = 0.05, animation = true, m = m, I_11 = 0, I_22 = 0, I_33 = 0) annotation(Placement(transformation(extent = {{-14, 14}, {-34, 34}}, origin = {4, -0}, rotation = 0), visible = true));
  UWmBody.UWParts.Body body2(r_CM = -0.2 * jointUPS.eAxis_ia, cylinderDiameter = 0.05, animation = true, m = m, I_11 = 0, I_22 = 0, I_33 = 0) annotation(Placement(transformation(extent = {{32, 14}, {52, 34}}, origin = {-2, -0}, rotation = 0), visible = true));
  UWmBody.UWParts.FixedTranslation rod2(r = {0, 0.3, 0}, width = 0.01, animation = false) annotation(Placement(transformation(origin = {-40, 85}, extent = {{10, -10}, {-10, 10}}, rotation = 270), visible = true));
  Modelica.Mechanics.Translational.Components.Damper damper1(d = 3) annotation(Placement(transformation(extent = {{0, 24}, {20, 4}}, origin = {0, -4}, rotation = 0), visible = true));
  UWmBody.UWJoints.Revolute revolute2(phi(fixed = true), w(fixed = true)) annotation(Placement(transformation(extent = {{-20, -40}, {0, -20}})));
  UWmBody.UWParts.BodyBox bodyBox2(r = {0.7, 0, 0}) annotation(Placement(transformation(extent = {{20, -40}, {40, -20}})));
  UWmBody.UWParts.FixedTranslation rod3(width = 0.01, r = {0, -0.9, 0.3}, animation = false) annotation(Placement(transformation(origin = {-40, -45}, extent = {{-10, -10}, {10, 10}}, rotation = 270), visible = true));
  UWmBody.UWParts.FixedTranslation rod4(width = 0.01, r = {0, 0.3, 0.3}, animation = false) annotation(Placement(transformation(origin = {-40, -15}, extent = {{10, -10}, {-10, 10}}, rotation = 270), visible = true));
  Modelica.Mechanics.Translational.Components.Damper damper2(d = 3) annotation(Placement(transformation(extent = {{0, -76}, {20, -96}}, origin = {0, -4}, rotation = 0), visible = true));
  UWmBody.UWForces.LineForceWithTwoMasses lineForceWithTwoMasses(L_a = 0.2, L_b = 0.2, cylinderLength_a = 0.2, cylinderLength_b = 1.2, massDiameterFaction = 2.2, m_a = m, m_b = m) annotation(Placement(transformation(extent = {{0, -50}, {20, -70}})));
equation
  connect(jointUPS.bearing, damper1.flange_a) annotation(Line(points = {{6, 30}, {6, 20}, {0, 20}, {-0, 10}}, color = {0, 127, 0}, visible = true));
  connect(jointUPS.axis, damper1.flange_b) annotation(Line(points = {{14, 30}, {14, 20}, {20, 20}, {20, 10}}, color = {0, 127, 0}, visible = true));
  connect(jointUPS.frame_ib, body2.frame_a) annotation(Line(points = {{-4, 4}, {-4, -2}, {8, -2}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {22, 26}));
  connect(world.frame_b, rod2.frame_a) annotation(Line(points = {{-13.333, -1.667}, {6.667, -1.667}, {6.667, 3.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-46.667, 71.667}));
  connect(world.frame_b, rod1.frame_a) annotation(Line(points = {{-13.333, 1.667}, {6.667, 1.667}, {6.667, -3.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-46.667, 68.333}));
  connect(rod2.frame_b, revolute1.frame_a) annotation(Line(points = {{-38, 93}, {-38, 98}, {-28, 98}, {-28, 68}, {-18, 68}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-2, 2}));
  connect(revolute1.frame_b, bodyBox1.frame_a) annotation(Line(points = {{0, 70}, {20, 70}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(bodyBox1.frame_b, jointUPS.frame_b) annotation(Line(points = {{36, 70}, {46, 70}, {46, 40}, {16, 40}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {4, -0}));
  connect(body1.frame_a, jointUPS.frame_ia) annotation(Line(points = {{-8, -2}, {4, -2}, {4, 4}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-2, 26}));
  connect(rod1.frame_b, jointUPS.frame_a) annotation(Line(points = {{-13.333, 3.333}, {-13.333, -1.667}, {26.667, -1.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-26.667, 41.667}));
  connect(rod4.frame_b, revolute2.frame_a) annotation(Line(points = {{-40, -7}, {-40, -2}, {-30, -2}, {-30, -32}, {-20, -32}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, 2}));
  connect(revolute2.frame_b, bodyBox2.frame_a) annotation(Line(points = {{0, -30}, {20, -30}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(world.frame_b, rod4.frame_a) annotation(Line(points = {{-62, 68}, {-52, 68}, {-52, -32}, {-42, -32}, {-42, -27}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {2, 2}));
  connect(rod3.frame_a, rod4.frame_a) annotation(Line(points = {{0, -5}, {0, 5}}, thickness = 0.5, visible = true, color = {95, 95, 95}, origin = {-40, -30}));
  connect(lineForceWithTwoMasses.frame_a, rod3.frame_b) annotation(Line(points = {{26.667, -1.667}, {-13.333, -1.667}, {-13.333, 3.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-26.667, -58.333}));
  connect(lineForceWithTwoMasses.frame_b, bodyBox2.frame_b) annotation(Line(points = {{24, -60}, {54, -60}, {54, -30}, {44, -30}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-4, 0}));
  connect(lineForceWithTwoMasses.flange_b, damper2.flange_b) annotation(Line(points = {{16, -71}, {20, -71}, {20, -90}}, color = {0, 127, 0}, visible = true));
  connect(lineForceWithTwoMasses.flange_a, damper2.flange_a) annotation(Line(points = {{4, -71}, {0, -71}, {0, -90}}, color = {0, 127, 0}, visible = true));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Difference in cut forces", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = body_f_diff[1], legend = "Difference in cut-forces in bodyBox1 and bodyBox2 along x axis"), Curve(x = time, y = body_f_diff[2], legend = "Difference in cut-forces in bodyBox1 and bodyBox2 along y axis"), Curve(x = time, y = body_f_diff[3], legend = "Difference in cut-forces in bodyBox1 and bodyBox2 along z axis")}), SubPlot(curves = {Curve(x = time, y = rod_f_diff[1], legend = "Difference of cut-forces in rod1 and rod3 along x axis"), Curve(x = time, y = rod_f_diff[2], legend = "Difference of cut-forces in rod1 and rod3 along y axis"), Curve(x = time, y = rod_f_diff[3], legend = "Difference of cut-forces in rod1 and rod3 along z axis")})})})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})), experiment(StopTime = 3), Documentation(info = "<html>
<p>
It is demonstrated how to implement line force components
that shall have mass properties. Two alternative implementations
are given:
</p>
<ul>
<li> With <a href=\"modelica://UWmBody.Joints.Assemblies.JointUPS\">JointUPS</a>:<br>
     UWmBody.Joints.Assemblies.JointUPS is an aggregation
     of a universal, a prismatic and a spherical joint that approximates
     a real force component, such as a hydraulic cylinder. At the two
     frames of the prismatic joint (frame_ia, frame_ib of jointUPS)
     two bodies are attached. The parameters are selected such that
     the center of masses of the two bodies are located on the line
     connecting frame_a and frame_b of the jointUPS component.
     Both bodies have the same mass and the inertia tensor is set to zero,
     i.e., the two bodies are treated as point masses.</li>
<li> With <a href=\"modelica://UWmBody.Forces.LineForceWithTwoMasses\">LineForceWithTwoMasses</a>:<br>
     UWmBody.Forces.LineForceWithTwoMasses is a line force component
     with the built-in property that two point masses are located
     on the line on which the line force is acting.
     The parameters are selected in such a way that the same
     system as with the jointUPS component is described.</li>
</ul>
<p>
In both cases, a linear 1-dimensional translational damper from the
Modelica.Mechanics.Translational library is used as
line force between the two attachment points. Simulate
this system and plot the differences of the cut forces at both sides
of the line force component (\"rod_f_diff\" and \"body_f_diff\").
Both vectors should be zero
(depending on the chosen relative tolerance of the integration,
the difference is in the order of 1.e-10 ... 1.e-15).
</p>
<p>
Note, that the implementation with the LineForceWithTwoMasses
component is simpler and more convenient.
An animation of this simulation is shown in the figure below.
The system on the left side in the front is the animation with
the LineForceWithTwoMasses component whereas the system on the right
side in the back is the animation with the JointUPS component.
</p>

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/LineForceWithTwoMasses2.png\">

</html>"));
end LineForceWithTwoMasses;
