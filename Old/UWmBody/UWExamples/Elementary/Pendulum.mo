within UWmBody.UWExamples.Elementary;

model Pendulum "Simple pendulum with one revolute joint and one body"
  extends Modelica.Icons.Example;
  inner UWmBody.UWWorld world(gravityType = UWmBody.UWTypes.GravityTypes.UniformGravity) annotation(Placement(transformation(extent = {{-60, 0}, {-40, 20}}, origin = {10, -10}, rotation = 0), visible = true));
  UWmBody.UWJoints.Revolute rev(n = {0, 0, 1}, useAxisFlange = true, phi(fixed = true), w(fixed = true)) annotation(Placement(transformation(extent = {{-20, 0}, {0, 20}}, origin = {10, -10}, rotation = 0), visible = true));
  Modelica.Mechanics.Rotational.Components.Damper damper(d = 0.1) annotation(Placement(transformation(extent = {{-20, 40}, {0, 60}}, origin = {10, -10}, rotation = 0), visible = true));
  UWmBody.UWParts.Body body(m = 1.0, r_CM = {0.5, 0, 0}) annotation(Placement(transformation(extent = {{20, 0}, {40, 20}}, origin = {10, -10}, rotation = 0), visible = true));
equation
  connect(world.frame_b, rev.frame_a) annotation(Line(points = {{-40, 10}, {-20, 10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, -10}));
  connect(damper.flange_b, rev.axis) annotation(Line(points = {{-6, 46}, {4, 46}, {4, 26}, {-16, 26}, {-16, 16}}, visible = true, origin = {16, -6}, color = {64, 64, 64}));
  connect(rev.support, damper.flange_a) annotation(Line(points = {{-14, 16}, {-14, 26}, {-28, 26}, {-28, 46}, {-18, 46}}, visible = true, origin = {8, -6}, color = {64, 64, 64}));
  connect(body.frame_a, rev.frame_b) annotation(Line(points = {{20, 10}, {0, 10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, -10}));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Pendulum angle", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = rev.phi, legend = "Angle of pendulum")}), SubPlot(curves = {Curve(x = time, y = damper.tau, legend = "Torque applied to revolute joint from damper")})})})), experiment(StopTime = 5), Documentation(info = "<html>
<p>
This simple model demonstrates that by just dragging components
default animation is defined that shows the structure of the
assembled system.

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/Pendulum.png\"
ALT=\"model Examples.Elementary.Pendulum\">
</html>"));
end Pendulum;
