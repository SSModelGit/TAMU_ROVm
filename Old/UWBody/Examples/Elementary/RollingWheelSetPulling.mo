within UWBody.Examples.Elementary;

model RollingWheelSetPulling "Rolling wheel set that is pulled by a force"
  extends Modelica.Icons.Example;
  UWBody.Forces.WorldForce force(animation = false) annotation(Placement(transformation(extent = {{-20, 60}, {0, 80}}, origin = {0, -10}, rotation = 0), visible = true));
  UWBody.Visualizers.Ground ground(length = 3) annotation(Placement(transformation(extent = {{-20, -40}, {0, -20}}, origin = {-10, -30}, rotation = 0), visible = true));
  inner UWBody.World world(label2 = "z", n = {0, 0, -1}) annotation(Placement(transformation(extent = {{-60, 0}, {-40, 20}}, origin = {-10, -30}, rotation = 0), visible = true));
  UWBody.Parts.RollingWheelSet wheelSet(wheelRadius = 0.1, wheelMass = 0.5, wheel_I_axis = 0.01, wheel_I_long = 0.02, wheelDistance = 0.5, x(start = 0.1, fixed = true), y(start = 0.1, fixed = true), phi(fixed = true), theta1(fixed = true), theta2(fixed = true), der_theta1(fixed = true), der_theta2(fixed = true)) annotation(Placement(transformation(extent = {{-20, 0}, {0, 20}}, origin = {-10, -30}, rotation = 0), visible = true));
  UWBody.Parts.Body body(m = 0.01, r_CM = {0, 0, 0}, animation = false) annotation(Placement(transformation(extent = {{42, 20}, {62, 40}}, origin = {-2, -20}, rotation = 0), visible = true));
  Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(table = [0, 1, 0, 0; 1, 1, 0, 0; 2, 0, 2, 0; 3, 0, 2, 0]) annotation(Placement(transformation(extent = {{-80, 70}, {-60, 90}}, origin = {20, -20}, rotation = 0), visible = true));
  UWBody.Parts.FixedTranslation fixedTranslation(r = {0.2, 0, 0}, animation = true, width = 0.04) annotation(Placement(transformation(extent = {{0, 20}, {20, 40}}, origin = {-10, -20}, rotation = 0), visible = true));
  UWBody.Visualizers.FixedShape shape(final lengthDirection = {0, 1, 0}, final widthDirection = {1, 0, 0}, final shapeType = "pipe", final r_shape = {0, -wheelSet.wheelWidth, 0}, final length = 2 * wheelSet.wheelWidth, final width = 2 * wheelSet.wheelRadius, final height = 2 * wheelSet.wheelRadius, final color = {0, 128, 255}, final extra = 0.8) annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {50, 40}, rotation = 0), visible = true));
equation
  connect(combiTimeTable.y, force.force) annotation(Line(points = {{-8.5, 0}, {8.5, 0}}, color = {1, 37, 163}, visible = true, origin = {-30.5, 60}));
  connect(fixedTranslation.frame_a, wheelSet.frameMiddle) annotation(Line(points = {{6.667, 10}, {-3.333, 10}, {-3.333, -20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-16.667, 0}));
  connect(fixedTranslation.frame_b, body.frame_a) annotation(Line(points = {{-15, 0}, {15, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {25, 10}));
  connect(force.frame_b, fixedTranslation.frame_b) annotation(Line(points = {{14, 80}, {34, 80}, {34, 30}, {24, 30}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-14, -20}));
  connect(shape.frame_a, fixedTranslation.frame_b) annotation(Line(points = {{54, 60}, {34, 60}, {34, 30}, {24, 30}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-14, -20}));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Forces and position", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = force.force[1], legend = "Force applied along the x axis in the world coordinate system"), Curve(x = time, y = force.force[2], legend = "Force applied along the y axis in the world coordinate system")}), SubPlot(curves = {Curve(x = time, y = wheelSet.frameMiddle.f[1], legend = "Force applied along the x axis in the middle wheel set coordinate system"), Curve(x = time, y = wheelSet.frameMiddle.f[2], legend = "Force applied along the y axis in the middle wheel set coordinate system")}), SubPlot(curves = {Curve(x = time, y = wheelSet.frameMiddle.r_0[1], legend = "Position of middle frame of wheels along x axis"), Curve(x = time, y = wheelSet.frameMiddle.r_0[2], legend = "Position of middle frame of wheels along y axis")})})})), experiment(StopTime = 3), Documentation(info = "<html>
<p>
Demonstrates how a RollingWheelSet (two wheels rigidly coupled together) is rolling
on ground when pulled by an external force..
</p>
</html>"));
end RollingWheelSetPulling;
