within UWBody.Examples.Elementary;

model DoublePendulumInitTip "Demonstrate how to initialize a double pendulum so that its tip starts at a predefined position"
  extends Modelica.Icons.Example;
  inner World world annotation(Placement(transformation(extent = {{-100, 0}, {-80, 20}}, origin = {0, -10}, rotation = 0), visible = true));
  Joints.Revolute revolute1(useAxisFlange = true) annotation(Placement(transformation(extent = {{-60, 0}, {-40, 20}}, origin = {0, -10}, rotation = 0), visible = true));
  Rotational.Components.Damper damper(d = 0.1) annotation(Placement(transformation(extent = {{-60, 40}, {-40, 60}}, origin = {0, -10}, rotation = 0), visible = true));
  Parts.BodyBox boxBody1(r = {0.5, 0, 0}, width = 0.06) annotation(Placement(transformation(extent = {{-22, 0}, {-2, 20}}, origin = {0, -10}, rotation = 0), visible = true));
  Joints.Revolute revolute2 annotation(Placement(transformation(extent = {{20, 0}, {40, 20}}, origin = {0, -10}, rotation = 0), visible = true));
  Parts.BodyBox boxBody2(r = {0.5, 0, 0}, width = 0.06) annotation(Placement(transformation(extent = {{62, 0}, {82, 20}}, origin = {-2, -10}, rotation = 0), visible = true));
  UWBody.Joints.FreeMotionScalarInit freeMotionScalarInit(use_r = true, r_rel_a_1(start = 0.7, fixed = true), r_rel_a_2(start = 0.3, fixed = true), use_v = true, v_rel_a_1(fixed = true), v_rel_a_2(fixed = true)) annotation(Placement(transformation(extent = {{-20, -40}, {0, -20}}, origin = {0, -0}, rotation = 0), visible = true));
equation
  connect(damper.flange_b, revolute1.axis) annotation(Line(points = {{-46, 48}, {-36, 48}, {-36, 28}, {-56, 28}, {-56, 18}}, visible = true, origin = {6, -8}, color = {64, 64, 64}));
  connect(revolute1.support, damper.flange_a) annotation(Line(points = {{-56, 18}, {-56, 28}, {-70, 28}, {-70, 48}, {-60, 48}}, visible = true, origin = {0, -8}, color = {64, 64, 64}));
  connect(revolute1.frame_b, boxBody1.frame_a) annotation(Line(points = {{-40, 10}, {-22, 10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -10}));
  connect(revolute2.frame_b, boxBody2.frame_a) annotation(Line(points = {{-10, -0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {50, -0}));
  connect(boxBody1.frame_b, revolute2.frame_a) annotation(Line(points = {{-2, 10}, {20, 10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -10}));
  connect(world.frame_b, revolute1.frame_a) annotation(Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -10}));
  connect(world.frame_b, freeMotionScalarInit.frame_a) annotation(Line(points = {{-80, 10}, {-70, 10}, {-70, -20}, {-20, -20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -10}));
  connect(freeMotionScalarInit.frame_b, boxBody2.frame_b) annotation(Line(points = {{-2, -20}, {88, -20}, {88, 10}, {78, 10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {2, -10}));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Initial conditions", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = boxBody2.frame_b.r_0[1], legend = "Position of end point of pendulum along the x axis"), Curve(x = time, y = boxBody2.frame_b.r_0[2], legend = "Position of end point of pendulum along the y axis")}, range = Range(xmin = -0.02177068214804064, xmax = 0.05805515239477503, ymin = -0.02844285714285699, ymax = 0.8255857142857144)), SubPlot(curves = {Curve(x = time, y = der(boxBody2.r_0[1]), legend = "Velocity of end point of pendulum along the x axis"), Curve(x = time, y = der(boxBody2.r_0[2]), legend = "Velocity of end point of pendulum along the y axis")}, range = Range(xmin = -0.02177068214804064, xmax = 0.05805515239477503, ymin = auto, ymax = auto))}), Plot(name = "End point position", subPlots = {SubPlot(curves = {Curve(x = boxBody2.frame_b.r_0[1], y = boxBody2.frame_b.r_0[2], legend = "End point position of pendulum in the x and y plane")})})})), experiment(StopTime = 5), Documentation(info = "<html>
<p>
This example demonstrates at hand of a double pendulum,
how no-standard initialization can be defined:
The absolute position of the pendulum tip, and its absolute speed
shall be initially defined. This can be performed with the
<a href=\"modelica://UWBody.Joints.FreeMotionScalarInit\">Joints.FreeMotionScalarInit</a>
joint that allows to initialize individual elements of its relative vectors.
In this case, the x-, and y-coordinates of the relative position vector
(visualized by the yellow arrow in the figure below) and of its
derivative shall have a defined value at initial time.
The configuration of the double pendulum at the initial time is
shown below, where the tip position is required to have the coordinates
x=0.7, y=0.3.
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/DoublePendulumInitTip.png\">
</blockquote>

</html>"));
end DoublePendulumInitTip;
