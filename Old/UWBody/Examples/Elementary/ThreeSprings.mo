within UWBody.Examples.Elementary;

model ThreeSprings "3-dim. springs in series and parallel connection"
  extends Modelica.Icons.Example;
  parameter Boolean animation = true "= true, if animation shall be enabled";
  inner UWBody.World world(animateWorld = animation) annotation(Placement(transformation(extent = {{-60, 20}, {-40, 40}}, origin = {0, 10}, rotation = 0), visible = true));
  UWBody.Parts.Body body1(animation = animation, r_CM = {0, -0.2, 0}, m = 0.8, I_11 = 0.1, I_22 = 0.1, I_33 = 0.1, sphereDiameter = 0.2, r_0(start = {0.5, -0.3, 0}, each fixed = true), v_0(each fixed = true), angles_fixed = true, w_0_fixed = true) annotation(Placement(transformation(origin = {40, -50}, extent = {{-10, 10}, {10, -10}}, rotation = 270), visible = true));
  UWBody.Parts.FixedTranslation bar1(animation = animation, r = {0.3, 0, 0}) annotation(Placement(transformation(extent = {{-20, 20}, {0, 40}}, origin = {20, 10}, rotation = 0), visible = true));
  UWBody.Forces.Spring spring1(lineForce(r_rel_0(start = {-0.2, -0.2, 0.2})), s_unstretched = 0.1, width = 0.1, coilWidth = 0.005, numberOfWindings = 5, c = 20, animation = animation) annotation(Placement(transformation(origin = {40, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 270), visible = true));
  UWBody.Parts.FixedTranslation bar2(animation = animation, r = {0, 0, 0.3}) annotation(Placement(transformation(origin = {-20, 20}, extent = {{10, -10}, {-10, 10}}, rotation = 90), visible = true));
  UWBody.Forces.Spring spring2(s_unstretched = 0.1, width = 0.1, coilWidth = 0.005, numberOfWindings = 5, c = 40, animation = animation) annotation(Placement(transformation(origin = {40, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 270), visible = true));
  UWBody.Forces.Spring spring3(s_unstretched = 0.1, width = 0.1, coilWidth = 0.005, numberOfWindings = 5, c = 20, animation = animation, fixedRotationAtFrame_b = true) annotation(Placement(transformation(extent = {{-20, -42}, {0, -22}}, origin = {20, 32}, rotation = 0), visible = true));
equation
  connect(world.frame_b, bar1.frame_a) annotation(Line(points = {{-20, 0}, {20, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-20, 40}));
  connect(world.frame_b, bar2.frame_a) annotation(Line(points = {{-13.333, 3.333}, {6.667, 3.333}, {6.667, -6.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-26.667, 36.667}));
  connect(bar1.frame_b, spring1.frame_a) annotation(Line(points = {{-13.333, 3.333}, {6.667, 3.333}, {6.667, -6.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {33.333, 36.667}));
  connect(bar2.frame_b, spring3.frame_a) annotation(Line(points = {{-6.667, 6.667}, {-6.667, -3.333}, {13.333, -3.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-13.333, 3.333}));
  connect(spring2.frame_b, body1.frame_a) annotation(Line(points = {{30, -40}, {30, -60}, {30, -50}}, thickness = 0.5, visible = true, origin = {10, 10}, color = {95, 95, 95}));
  connect(spring3.frame_b, spring1.frame_b) annotation(Line(points = {{10, -10}, {30, -10}, {30, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, 10}));
  connect(spring2.frame_a, spring1.frame_b) annotation(Line(points = {{30, -20}, {30, -21}, {30, -21}, {30, -14}, {30, 0}, {30, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, 10}));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Spring elongations and forces", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = spring1.spring.f, legend = "Force in spring1"), Curve(x = time, y = spring2.spring.f, legend = "Force in spring2"), Curve(x = time, y = spring3.spring.f, legend = "Force in spring3")}), SubPlot(curves = {Curve(x = time, y = spring1.spring.s_rel, legend = "Elongation of spring1"), Curve(x = time, y = spring2.spring.s_rel, legend = "Elongation of spring2"), Curve(x = time, y = spring3.spring.s_rel, legend = "Elongation of spring3")})})})), experiment(StopTime = 10), Documentation(info = "<html>
<p>
This example demonstrates that <b>3-dimensional line force</b> elements
(here: UWBody.Forces.Spring elements) can be connected together
in <b>series</b> without having a body with mass at the
connection point (as usually required by multi-body programs).
This is advantageous since stiff systems can be avoided, say, due to
a stiff spring and a small mass at the connection point.
</p>

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/ThreeSprings.png\"
ALT=\"model Examples.Elementary.ThreeSprings\">

<p>
For a more thorough explanation, see
<a href=\"modelica://UWBody.UsersGuide.Tutorial.ConnectionOfLineForces\">MultiBody.UsersGuide.Tutorial.ConnectionOfLineForces</a>.
</p>
</html>"));
end ThreeSprings;
