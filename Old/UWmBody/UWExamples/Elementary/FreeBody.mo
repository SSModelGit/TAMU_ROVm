within UWmBody.UWExamples.Elementary;

model FreeBody "Free flying body attached by two springs to environment"
  extends Modelica.Icons.Example;
  parameter Boolean animation = true "= true, if animation shall be enabled";
  inner UWmBody.UWWorld world annotation(Placement(transformation(extent = {{-60, 20}, {-40, 40}}, origin = {-0, 0}, rotation = 0), visible = true));
  UWmBody.UWParts.FixedTranslation bar2(r = {0.8, 0, 0}, animation = false) annotation(Placement(transformation(extent = {{0, 20}, {20, 40}}, origin = {-0, 0}, rotation = 0), visible = true));
  UWmBody.UWForces.Spring spring1(width = 0.1, coilWidth = 0.005, numberOfWindings = 5, c = 20, s_unstretched = 0) annotation(Placement(transformation(origin = {-20, -0}, extent = {{-10, -10}, {10, 10}}, rotation = 270), visible = true));
  UWmBody.UWParts.BodyShape body(m = 1, I_11 = 1, I_22 = 1, I_33 = 1, r = {0.4, 0, 0}, r_CM = {0.2, 0, 0}, width = 0.05, r_0(start = {0.2, -0.5, 0.1}, each fixed = true), v_0(each fixed = true), angles_fixed = true, w_0_fixed = true, angles_start = {0.174532925199433, 0.174532925199433, 0.174532925199433}) annotation(Placement(transformation(extent = {{0, -40}, {20, -20}}, origin = {-0, 0}, rotation = 0), visible = true));
  UWmBody.UWForces.Spring spring2(c = 20, s_unstretched = 0, width = 0.1, coilWidth = 0.005, numberOfWindings = 5) annotation(Placement(transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 270), visible = true));
equation
  connect(bar2.frame_a, world.frame_b) annotation(Line(points = {{0, 30}, {-40, 30}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(spring1.frame_b, body.frame_a) annotation(Line(points = {{-6.667, 13.333}, {-6.667, -6.667}, {13.333, -6.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-13.333, -23.333}));
  connect(bar2.frame_b, spring2.frame_a) annotation(Line(points = {{-13.333, 6.667}, {6.667, 6.667}, {6.667, -13.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {33.333, 23.333}));
  connect(spring1.frame_a, world.frame_b) annotation(Line(points = {{6.667, -13.333}, {6.667, 6.667}, {-13.333, 6.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-26.667, 23.333}));
  connect(body.frame_b, spring2.frame_b) annotation(Line(points = {{-13.333, -6.667}, {6.667, -6.667}, {6.667, 13.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {33.333, -23.333}));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Body position and spring forces", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = body.body.frame_a.r_0[1], legend = "Body position along x axis"), Curve(x = time, y = body.body.frame_a.r_0[2], legend = "Body position along y axis"), Curve(x = time, y = body.body.frame_a.r_0[3], legend = "Body position along z axis")}), SubPlot(curves = {Curve(x = time, y = spring1.spring.f, legend = "Force from left spring"), Curve(x = time, y = spring2.spring.f, legend = "Force from right spring")})})})), experiment(StopTime = 10), Documentation(info = "<html>
<p>
This example demonstrates:
</p>
<ul>
<li>The animation of spring and damper components</li>
<li>A body can be freely moving without any connection to a joint.
    In this case body coordinates are used automatically as
    states (whenever joints are present, it is first tried to
    use the generalized coordinates of the joints as states).</li>
<li>If a body is freely moving, the initial position and velocity of the body
    can be defined with the \"Initialization\" menu as shown with the
    body \"body1\" in the left part (click on \"Initialization\").</li>
</ul>

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/FreeBody.png\"
ALT=\"model Examples.Elementary.FreeBody\">
</html>"));
end FreeBody;
