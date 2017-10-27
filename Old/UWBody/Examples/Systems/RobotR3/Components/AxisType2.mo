within UWBody.Examples.Systems.RobotR3.Components;

model AxisType2 "Axis model of the r3 joints 4,5,6"
  parameter Real kp = 10 "Gain of position controller" annotation(Dialog(group = "Controller"));
  parameter Real ks = 1 "Gain of speed controller" annotation(Dialog(group = "Controller"));
  parameter SI.Time Ts = 0.01 "Time constant of integrator of speed controller" annotation(Dialog(group = "Controller"));
  parameter Real k = 1.1616 "Gain of motor" annotation(Dialog(group = "Motor"));
  parameter Real w = 4590 "Time constant of motor" annotation(Dialog(group = "Motor"));
  parameter Real D = 0.6 "Damping constant of motor" annotation(Dialog(group = "Motor"));
  parameter SI.Inertia J(min = 0) = 0.0013 "Moment of inertia of motor" annotation(Dialog(group = "Motor"));
  parameter Real ratio = -105 "Gear ratio" annotation(Dialog(group = "Gear"));
  parameter SI.Torque Rv0 = 0.4 "Viscous friction torque at zero velocity in [Nm]" annotation(Dialog(group = "Gear"));
  parameter Real Rv1(unit = "N.m.s/rad") = 0.13 / 160 "Viscous friction coefficient in [Nms/rad]" annotation(Dialog(group = "Gear"));
  parameter Real peak = 1 "Maximum static friction torque is peak*Rv0 (peak >= 1)" annotation(Dialog(group = "Gear"));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange annotation(Placement(transformation(extent = {{90, -10}, {110, 10}})));
  replaceable GearType2 gear(Rv0 = Rv0, Rv1 = Rv1, peak = peak, i = ratio) annotation(Placement(transformation(extent = {{0, -10}, {20, 10}})));
  Motor motor(J = J, k = k, w = w, D = D) annotation(Placement(transformation(extent = {{-30, -10}, {-10, 10}})));
  RobotR3.Components.Controller controller(kp = kp, ks = ks, Ts = Ts, ratio = ratio) annotation(Placement(transformation(extent = {{-70, -10}, {-50, 10}})));
  UWBody.Examples.Systems.RobotR3.Components.AxisControlBus axisControlBus annotation(Placement(transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation(Placement(transformation(extent = {{30, 60}, {50, 80}})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(Placement(transformation(origin = {40, 50}, extent = {{10, 10}, {-10, -10}}, rotation = 180)));
  Modelica.Mechanics.Rotational.Sensors.AccSensor accSensor annotation(Placement(transformation(extent = {{30, 20}, {50, 40}})));
  Modelica.Mechanics.Rotational.Components.InitializeFlange initializeFlange(stateSelect = StateSelect.prefer) annotation(Placement(transformation(extent = {{-40, -60}, {-20, -40}})));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(transformation(extent = {{-65, -65}, {-55, -55}})));
equation
  connect(gear.flange_b, flange) annotation(Line(points = {{20, 0}, {100, 0}}));
  connect(gear.flange_b, angleSensor.flange) annotation(Line(points = {{20, 0}, {20, 70}, {30, 70}}));
  connect(gear.flange_b, speedSensor.flange) annotation(Line(points = {{20, 0}, {24, 0}, {24, 50}, {30, 50}}));
  connect(motor.flange_motor, gear.flange_a) annotation(Line(points = {{-10, 0}, {0, 0}}));
  connect(gear.flange_b, accSensor.flange) annotation(Line(points = {{20, 0}, {28, 0}, {28, 30}, {30, 30}}));
  connect(controller.axisControlBus, axisControlBus) annotation(Line(points = {{-60, -10}, {-60, -20}, {-95, -20}, {-95, -4}, {-100, -4}, {-100, 0}}, color = {255, 204, 51}, thickness = 0.5));
  connect(motor.axisControlBus, axisControlBus) annotation(Line(points = {{-12, -10}, {-12, -20}, {-95, -20}, {-95, -5}, {-100, -5}, {-100, 0}}, color = {255, 204, 51}, thickness = 0.5));
  connect(angleSensor.phi, axisControlBus.angle) annotation(Text(string = "%second", index = 1, extent = [6, 3; 6, 3]), Line(points = {{51, 70}, {70, 70}, {70, 84}, {-98, 84}, {-98, 9}, {-99.9, 9}, {-99.9, -0.1}}, color = {0, 0, 127}));
  connect(speedSensor.w, axisControlBus.speed) annotation(Text(string = "%second", index = 1, extent = [6, 3; 6, 3]), Line(points = {{51, 50}, {74, 50}, {74, 87}, {-99.9, 87}, {-99.9, -0.1}}, color = {0, 0, 127}));
  connect(accSensor.a, axisControlBus.acceleration) annotation(Text(string = "%second", index = 1, extent = [6, 3; 6, 3]), Line(points = {{51, 30}, {77, 30}, {77, 90}, {-102, 90}, {-102, -0.1}, {-99.9, -0.1}}, color = {0, 0, 127}));
  connect(axisControlBus.angle_ref, initializeFlange.phi_start) annotation(Text(string = "%first", index = -1, extent = [-6, 3; -6, 3]), Line(points = {{-99.9, -0.1}, {-99.9, -7}, {-97, -7}, {-97, -42}, {-42, -42}}));
  connect(axisControlBus.speed_ref, initializeFlange.w_start) annotation(Text(string = "%first", index = -1, extent = [-6, 3; -6, 3]), Line(points = {{-99.9, -0.1}, {-99, -0.1}, {-99, -50}, {-42, -50}}, color = {0, 0, 127}));
  connect(initializeFlange.flange, flange) annotation(Line(points = {{-20, -50}, {80, -50}, {80, 0}, {100, 0}}));
  connect(const.y, initializeFlange.a_start) annotation(Line(points = {{-54.5, -60}, {-48, -60}, {-48, -58}, {-42, -58}}, color = {0, 0, 127}));
  annotation(Documentation(info = "<html>
<p>
The axis model consists of the <b>controller</b>, the <b>motor</b> including current
controller and the <b>gearbox</b> including gear elasticity and bearing friction.
The only difference to the axis model of joints 4,5,6 (= model axisType2) is
that elasticity and damping in the gear boxes are not neglected.
</p>
<p>
The input signals of this component are the desired angle and desired angular
velocity of the joint. The reference signals have to be \"smooth\" (position
has to be differentiable at least 2 times). Otherwise, the gear elasticity
leads to significant oscillations.
</p>
<p>
Default values of the parameters are given for the axis of joint 1.
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, -50}, {100, 50}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 57}, {150, 97}}, textString = "%name")}));
end AxisType2;
