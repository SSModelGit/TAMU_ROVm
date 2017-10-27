within UWBody.Examples.Systems.RobotR3.Components;

model Controller "P-PI cascade controller for one axis"
  parameter Real kp = 10 "Gain of position controller";
  parameter Real ks = 1 "Gain of speed controller";
  parameter SI.Time Ts = 0.01 "Time constant of integrator of speed controller";
  parameter Real ratio = 1 "Gear ratio of gearbox";
  Modelica.Blocks.Math.Gain gain1(k = ratio) annotation(Placement(transformation(extent = {{-70, 0}, {-50, 20}})));
  Modelica.Blocks.Continuous.PI PI(k = ks, T = Ts) annotation(Placement(transformation(extent = {{60, 0}, {80, 20}})));
  Modelica.Blocks.Math.Feedback feedback1 annotation(Placement(transformation(extent = {{-46, 0}, {-26, 20}})));
  Modelica.Blocks.Math.Gain P(k = kp) annotation(Placement(transformation(extent = {{-16, 0}, {4, 20}})));
  Modelica.Blocks.Math.Add3 add3(k3 = -1) annotation(Placement(transformation(extent = {{20, 0}, {40, 20}})));
  Modelica.Blocks.Math.Gain gain2(k = ratio) annotation(Placement(transformation(extent = {{-60, 40}, {-40, 60}})));
  UWBody.Examples.Systems.RobotR3.Components.AxisControlBus axisControlBus annotation(Placement(transformation(extent = {{-20, -120}, {20, -80}})));
equation
  connect(gain1.y, feedback1.u1) annotation(Line(points = {{-49, 10}, {-44, 10}}, color = {0, 0, 127}));
  connect(feedback1.y, P.u) annotation(Line(points = {{-27, 10}, {-18, 10}}, color = {0, 0, 127}));
  connect(P.y, add3.u2) annotation(Line(points = {{5, 10}, {18, 10}}, color = {0, 0, 127}));
  connect(gain2.y, add3.u1) annotation(Line(points = {{-39, 50}, {10, 50}, {10, 18}, {18, 18}}, color = {0, 0, 127}));
  connect(add3.y, PI.u) annotation(Line(points = {{41, 10}, {58, 10}}, color = {0, 0, 127}));
  connect(gain2.u, axisControlBus.speed_ref) annotation(Line(points = {{-62, 50}, {-90, 50}, {-90, -99.9}, {0.1, -99.9}}, color = {0, 0, 127}));
  connect(gain1.u, axisControlBus.angle_ref) annotation(Line(points = {{-72, 10}, {-80, 10}, {-80, -99.9}, {0.1, -99.9}}, color = {0, 0, 127}));
  connect(feedback1.u2, axisControlBus.motorAngle) annotation(Line(points = {{-36, 2}, {-36, -99.9}, {0.1, -99.9}}, color = {0, 0, 127}));
  connect(add3.u3, axisControlBus.motorSpeed) annotation(Line(points = {{18, 2}, {0.1, 2}, {0.1, -99.9}}, color = {0, 0, 127}));
  connect(PI.y, axisControlBus.current_ref) annotation(Line(points = {{81, 10}, {90, 10}, {90, -99.9}, {0.1, -99.9}}, color = {0, 0, 127}));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {235, 235, 235}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Rectangle(visible = true, lineColor = {10, 90, 224}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-30, 25}, {30, 55}}), Line(visible = true, points = {{-30, -40}, {-80, -40}, {-80, 40}, {-30, 40}}, color = {10, 90, 224}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Rectangle(visible = true, lineColor = {10, 90, 224}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-30, -55}, {30, -25}}), Line(visible = true, points = {{30, 40}, {80, 40}, {80, -40}, {30, -40}}, color = {10, 90, 224}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Text(visible = true, textColor = {64, 64, 64}, extent = {{-100, 110}, {100, 150}}, textString = "%name")}), Documentation(info = "<html>
<p>
This controller has an inner PI-controller to control the motor speed,
and an outer P-controller to control the motor position of one axis.
The reference signals are with respect to the gear-output, and the
gear ratio is used in the controller to determine the motor
reference signals. All signals are communicated via the
\"axisControlBus\".
</p>
</html>"));
end Controller;
