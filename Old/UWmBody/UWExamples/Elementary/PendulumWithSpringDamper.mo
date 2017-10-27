within UWmBody.UWExamples.Elementary;

model PendulumWithSpringDamper "Simple spring/damper/mass system"
  extends Modelica.Icons.Example;
  parameter Boolean animation = true "= true, if animation shall be enabled";
  inner UWmBody.UWWorld world(axisLength = 0.6) annotation(Placement(transformation(extent = {{-80, 20}, {-60, 40}}, origin = {0, -5}, rotation = 0), visible = true));
  UWmBody.UWParts.Body body1(m = 1, animation = animation, I_11 = 1, I_22 = 1, I_33 = 1, r_CM = {0, 0, 0}, cylinderDiameter = 0.05, sphereDiameter = 0.2) annotation(Placement(transformation(origin = {70, 25}, extent = {{10, -10}, {-10, 10}}, rotation = 180), visible = true));
  UWmBody.UWParts.FixedTranslation bar1(animation = animation, r = {0.3, 0, 0}) annotation(Placement(transformation(extent = {{-46, 20}, {-26, 40}}, origin = {-4, -5}, rotation = 0), visible = true));
  UWmBody.UWForces.Spring spring1(coilWidth = 0.01, numberOfWindings = 5, c = 20, s_unstretched = 0.2) annotation(Placement(transformation(extent = {{0, -46}, {20, -26}}, origin = {5, 11}, rotation = 0), visible = true));
  UWmBody.UWForces.Damper damper1(d = 1, length_a = 0.1, diameter_a = 0.08, animation = false) annotation(Placement(transformation(extent = {{0, -20}, {20, 0}}, origin = {5, 10}, rotation = 0), visible = true));
  UWmBody.UWJoints.Revolute revolute(phi(fixed = true), w(fixed = true)) annotation(Placement(transformation(extent = {{-12, 20}, {8, 40}}, origin = {2, -5}, rotation = 0), visible = true));
  UWmBody.UWJoints.Prismatic prismatic(boxWidth = 0.04, boxColor = {255, 65, 65}, s(fixed = true, start = 0.5), v(fixed = true)) annotation(Placement(transformation(extent = {{20, 20}, {40, 40}}, origin = {0, -5}, rotation = 0), visible = true));
equation
  connect(world.frame_b, bar1.frame_a) annotation(Line(points = {{-5, 0}, {5, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-55, 25}));
  connect(revolute.frame_a, bar1.frame_b) annotation(Line(points = {{10, 0}, {-10, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-20, 25}));
  connect(prismatic.frame_a, revolute.frame_b) annotation(Line(points = {{5, 0}, {-5, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {15, 25}));
  connect(damper1.frame_a, bar1.frame_b) annotation(Line(points = {{5, 5}, {-20, 5}, {-20, 30}, {-30, 30}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -5}));
  connect(damper1.frame_b, prismatic.frame_b) annotation(Line(points = {{25, 5}, {50, 5}, {50, 30}, {40, 30}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -5}));
  connect(spring1.frame_a, bar1.frame_b) annotation(Line(points = {{5, -20}, {-20, -20}, {-20, 30}, {-30, 30}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -5}));
  connect(spring1.frame_b, prismatic.frame_b) annotation(Line(points = {{25, -20}, {50, -20}, {50, 30}, {40, 30}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -5}));
  connect(body1.frame_a, prismatic.frame_b) annotation(Line(points = {{10, 0}, {-10, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {50, 25}));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "End point position", preferred = true, subPlots = {SubPlot(curves = {Curve(x = body1.r_0[1], y = body1.r_0[2], legend = "Pendulum end point position in the x-y plane")})}), Plot(name = "Spring and damper", subPlots = {SubPlot(curves = {Curve(x = time, y = spring1.spring.f, legend = "Force from spring affecting the elongation of the prismatic joint, depending on displacement"), Curve(x = time, y = prismatic.s, legend = "Elongation of the prismatic joint"), Curve(x = time, y = damper1.f, legend = "Force from damper affecting the prismatic joint depending on the elongation velocity")})})})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})), experiment(StopTime = 10), Documentation(info = "<html>
<p>
A body is attached on a revolute and prismatic joint.
A 3-dim. spring and a 3-dim. damper are connected between the body
and a point fixed in the world frame:
</p>

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/PendulumWithSpringDamper.png\"
ALT=\"model Examples.Elementary.PendulumWithSpringDamper\">
</html>"));
end PendulumWithSpringDamper;
