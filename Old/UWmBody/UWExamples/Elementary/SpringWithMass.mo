within UWmBody.UWExamples.Elementary;

model SpringWithMass "Point mass hanging on a spring"
  extends Modelica.Icons.Example;
  inner UWmBody.UWWorld world(animateGravity = false) annotation(Placement(transformation(extent = {{-40, 40}, {-20, 60}}, origin = {10, -20}, rotation = 0), visible = true));
  UWmBody.UWForces.Spring spring(s_unstretched = 0.2, m = 0.5, c = 40, width = 0.1, massDiameter = 0.07) annotation(Placement(transformation(origin = {20, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 270), visible = true));
  UWmBody.UWParts.Body body(r_0(start = {0, -0.3, 0}, each fixed = true), v_0(each fixed = true), angles_fixed = true, w_0_fixed = true, r_CM = {0, 0, 0}, m = 1) annotation(Placement(transformation(origin = {20, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 270), visible = true));
equation
  connect(world.frame_b, spring.frame_a) annotation(Line(points = {{-20, 50}, {10, 50}, {10, 40}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, -20}));
  connect(body.frame_a, spring.frame_b) annotation(Line(points = {{10, 10}, {10, 10}, {10, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, -20}));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Elongation of spring", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = spring.lineForce.length, legend = "Elongation of spring")}), SubPlot(curves = {Curve(x = time, y = spring.lengthFraction, legend = "Fraction of distance from frame_a to frame_b where mass in spring is located"), Curve(x = time, y = spring.m, legend = "Mass of springMass")})})})), experiment(StopTime = 5), Documentation(info = "<html>
<p>
This example shows that a force component may have a mass.
The 3-dimensional spring as used in this example, has an optional
point mass between the two points where the spring is attached.
In the animation, this point mass is represented by a small,
light blue, sphere.
</p>

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/SpringWithMass.png\"
ALT=\"model Examples.Elementary.SpringWithMass\">
</html>"));
end SpringWithMass;
