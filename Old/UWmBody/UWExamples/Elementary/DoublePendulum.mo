within UWmBody.UWExamples.Elementary;

model DoublePendulum "Simple double pendulum with two revolute joints and two bodies"
  extends Modelica.Icons.Example;
  inner UWmBody.UWWorld world annotation(Placement(transformation(extent = {{-88, 0}, {-68, 20}}, origin = {-2, -10}, rotation = 0), visible = true));
  UWmBody.UWJoints.Revolute revolute1(useAxisFlange = true, phi(fixed = true), w(fixed = true)) annotation(Placement(transformation(extent = {{-48, 0}, {-28, 20}}, origin = {-2, -10}, rotation = 0), visible = true));
  Modelica.Mechanics.Rotational.Components.Damper damper(d = 0.1) annotation(Placement(transformation(extent = {{-48, 40}, {-28, 60}}, origin = {-2, -10}, rotation = 0), visible = true));
  UWmBody.UWParts.BodyBox boxBody1(r = {0.5, 0, 0}, width = 0.06) annotation(Placement(transformation(extent = {{-10, 0}, {10, 20}}, origin = {-2, -10}, rotation = 0), visible = true));
  UWmBody.UWJoints.Revolute revolute2(phi(fixed = true), w(fixed = true)) annotation(Placement(transformation(extent = {{32, 0}, {52, 20}}, origin = {-2, -10}, rotation = 0), visible = true));
  UWmBody.UWParts.BodyBox boxBody2(r = {0.5, 0, 0}, width = 0.06) annotation(Placement(transformation(extent = {{74, 0}, {94, 20}}, origin = {-4, -10}, rotation = 0), visible = true));
equation
  connect(damper.flange_b, revolute1.axis) annotation(Line(points = {{-34, 48}, {-24, 48}, {-24, 28}, {-44, 28}, {-44, 18}}, visible = true, color = {64, 64, 64}, origin = {4, -8}));
  connect(revolute1.support, damper.flange_a) annotation(Line(points = {{-44, 18}, {-44, 28}, {-58, 28}, {-58, 48}, {-48, 48}}, visible = true, color = {64, 64, 64}, origin = {-2, -8}));
  connect(revolute1.frame_b, boxBody1.frame_a) annotation(Line(points = {{-28, 10}, {-10, 10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-2, -10}));
  connect(revolute2.frame_b, boxBody2.frame_a) annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {60, -0}));
  connect(boxBody1.frame_b, revolute2.frame_a) annotation(Line(points = {{10, 10}, {32, 10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-2, -10}));
  connect(world.frame_b, revolute1.frame_a) annotation(Line(points = {{-68, 10}, {-48, 10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-2, -10}));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "End point position", preferred = true, subPlots = {SubPlot(curves = {Curve(x = boxBody2.frame_b.r_0[1], y = boxBody2.frame_b.r_0[2], legend = "End point position of pendulum in the x and y plane")})})})), experiment(StopTime = 3), Documentation(info = "<html>
<p>
This example demonstrates that by using joint and body
elements animation is automatically available. Also the revolute
joints are animated. Note, that animation of every component
can be switched of by setting the first parameter <b>animation</b>
to <b>false</b> or by setting <b>enableAnimation</b> in the <b>world</b>
object to <b>false</b> to switch off animation of all components.
</p>

<table border=0 cellspacing=0 cellpadding=0><tr><td valign=\"top\">
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/DoublePendulum.png\"
ALT=\"model Examples.Elementary.DoublePendulum\">
</td></tr></table>

</html>"));
end DoublePendulum;
