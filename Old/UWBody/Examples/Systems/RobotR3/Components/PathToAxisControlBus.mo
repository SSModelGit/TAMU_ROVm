within UWBody.Examples.Systems.RobotR3.Components;

model PathToAxisControlBus "Map path planning to one axis control bus"
  extends Modelica.Blocks.Icons.Block;
  parameter Integer nAxis = 6 "Number of driven axis";
  parameter Integer axisUsed = 1 "Map path planning of axisUsed to axisControlBus";
  Modelica.Blocks.Interfaces.RealInput q[nAxis] annotation(Placement(transformation(extent = {{-140, 60}, {-100, 100}})));
  Modelica.Blocks.Interfaces.RealInput qd[nAxis] annotation(Placement(transformation(extent = {{-140, 10}, {-100, 50}})));
  Modelica.Blocks.Interfaces.RealInput qdd[nAxis] annotation(Placement(transformation(extent = {{-140, -50}, {-100, -10}})));
  AxisControlBus axisControlBus annotation(Placement(transformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
  Modelica.Blocks.Routing.RealPassThrough q_axisUsed annotation(Placement(transformation(extent = {{-40, 50}, {-20, 70}})));
  Modelica.Blocks.Routing.RealPassThrough qd_axisUsed annotation(Placement(transformation(extent = {{-40, 10}, {-20, 30}})));
  Modelica.Blocks.Routing.RealPassThrough qdd_axisUsed annotation(Placement(transformation(extent = {{-40, -30}, {-20, -10}})));
  Modelica.Blocks.Interfaces.BooleanInput moving[nAxis] annotation(Placement(transformation(extent = {{-140, -100}, {-100, -60}})));
  Modelica.Blocks.Routing.BooleanPassThrough motion_ref_axisUsed annotation(Placement(transformation(extent = {{-40, -70}, {-20, -50}})));
equation
  connect(q_axisUsed.u, q[axisUsed]) annotation(Line(points = {{-42, 60}, {-60, 60}, {-60, 80}, {-120, 80}}, color = {0, 0, 127}));
  connect(qd_axisUsed.u, qd[axisUsed]) annotation(Line(points = {{-42, 20}, {-80, 20}, {-80, 30}, {-120, 30}}, color = {0, 0, 127}));
  connect(qdd_axisUsed.u, qdd[axisUsed]) annotation(Line(points = {{-42, -20}, {-80, -20}, {-80, -30}, {-120, -30}}, color = {0, 0, 127}));
  connect(motion_ref_axisUsed.u, moving[axisUsed]) annotation(Line(points = {{-42, -60}, {-60, -60}, {-60, -80}, {-120, -80}}, color = {255, 0, 255}));
  connect(motion_ref_axisUsed.y, axisControlBus.motion_ref) annotation(Text(string = "%second", index = 1, extent = [6, 3; 6, 3]), Line(points = {{-19, -60}, {44, -60}, {44, -8}, {102, -8}, {102, -0.1}, {100.1, -0.1}}, color = {255, 0, 255}));
  connect(qdd_axisUsed.y, axisControlBus.acceleration_ref) annotation(Text(string = "%second", index = 1, extent = [6, 3; 6, 3]), Line(points = {{-19, -20}, {40, -20}, {40, -4}, {98, -4}, {98, -0.1}, {100.1, -0.1}}, color = {0, 0, 127}));
  connect(qd_axisUsed.y, axisControlBus.speed_ref) annotation(Text(string = "%second", index = 1, extent = [6, 3; 6, 3]), Line(points = {{-19, 20}, {20, 20}, {20, -0.1}, {100.1, -0.1}}, color = {0, 0, 127}));
  connect(q_axisUsed.y, axisControlBus.angle_ref) annotation(Text(string = "%second", index = 1, extent = [6, 3; 6, 3]), Line(points = {{-19, 60}, {40, 60}, {40, -0.1}, {100.1, -0.1}}, color = {0, 0, 127}));
  annotation(defaultComponentName = "pathToAxis1", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-100, 68}, {-24, 98}}, textString = "q"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-94, 16}, {-18, 46}}, textString = "qd"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-96, -46}, {-20, -16}}, textString = "qdd"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-2, -18}, {80, 20}}, textString = "%axisUsed"), Text(visible = true, textColor = {64, 64, 64}, extent = {{2, 28}, {76, 52}}, textString = "axis"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-94, -96}, {32, -70}}, textString = "moving")}), Documentation(info = "<html>
<p>
This model stores the 4 reference variables q, qd, qdd, moving from the path planning on the axis control bus.
</p>
</html>"));
end PathToAxisControlBus;
