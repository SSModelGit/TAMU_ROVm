within UWBody.Examples.Elementary;

model PointGravity "Two point masses in a point gravity field"
  extends Modelica.Icons.Example;
  inner UWBody.World world(mue = 1, gravitySphereDiameter = 0.1, gravityType = UWBody.Types.GravityTypes.PointGravity) annotation(Placement(transformation(extent = {{-20, -20}, {0, 0}})));
  UWBody.Parts.Body body1(m = 1, sphereDiameter = 0.1, I_11 = 0.1, I_22 = 0.1, I_33 = 0.1, r_0(start = {0, 0.6, 0}, each fixed = true), v_0(start = {1, 0, 0}, each fixed = true), angles_fixed = true, w_0_fixed = true, r_CM = {0, 0, 0}) annotation(Placement(transformation(extent = {{-20, 20}, {0, 40}})));
  UWBody.Parts.Body body2(m = 1, sphereDiameter = 0.1, I_11 = 0.1, I_22 = 0.1, I_33 = 0.1, r_0(start = {0.6, 0.6, 0}, each fixed = true), v_0(start = {0.6, 0, 0}, each fixed = true), angles_fixed = true, w_0_fixed = true, r_CM = {0, 0, 0}) annotation(Placement(transformation(extent = {{20, 20}, {40, 40}})));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Body positions", preferred = true, subPlots = {SubPlot(curves = {Curve(x = body1.r_0[1], y = body1.r_0[2]), Curve(x = body2.r_0[1], y = body2.r_0[2])})})})), experiment(StopTime = 5), Documentation(info = "<html>
<p>
This model demonstrates a point gravity field. Two bodies
are placed in the gravity field. The initial positions and velocities of
these bodies are selected such that one body rotates on a circle and
the other body rotates on an ellipse around the center of the
point gravity field.
</p>

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/PointGravity.png\"
ALT=\"model Examples.Elementary.PointGravity\">
</html>"));
end PointGravity;
