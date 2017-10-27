within UWBody.Examples.Systems.RobotR3.Components;

model Motor "Motor model including current controller of r3 motors"
  extends UWBody.Icons.MotorIcon;
  parameter SI.Inertia J(min = 0) = 0.0013 "Moment of inertia of motor";
  parameter Real k = 1.1616 "Gain of motor";
  parameter Real w = 4590 "Time constant of motor";
  parameter Real D = 0.6 "Damping constant of motor";
  parameter SI.AngularVelocity w_max = 315 "Maximum speed of motor";
  parameter SI.Current i_max = 9 "Maximum current of motor";
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_motor annotation(Placement(transformation(extent = {{90, -10}, {110, 10}})));
  Modelica.Electrical.Analog.Sources.SignalVoltage Vs annotation(Placement(transformation(origin = {-90, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 270)));
  Modelica.Electrical.Analog.Ideal.IdealOpAmp diff annotation(Placement(transformation(extent = {{-64, 15}, {-44, 35}})));
  Modelica.Electrical.Analog.Ideal.IdealOpAmp power annotation(Placement(transformation(extent = {{16, 15}, {36, 35}})));
  Modelica.Electrical.Analog.Basic.EMF emf(k = k, useSupport = false) annotation(Placement(transformation(extent = {{46, -10}, {66, 10}})));
  Modelica.Electrical.Analog.Basic.Inductor La(L = 250 / (2 * D * w)) annotation(Placement(transformation(origin = {56, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
  Modelica.Electrical.Analog.Basic.Resistor Ra(R = 250) annotation(Placement(transformation(origin = {56, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
  Modelica.Electrical.Analog.Basic.Resistor Rd2(R = 100) annotation(Placement(transformation(extent = {{-86, 22}, {-71, 38}})));
  Modelica.Electrical.Analog.Basic.Capacitor C(C = 0.004 * D / w) annotation(Placement(transformation(extent = {{-14, 36}, {6, 56}})));
  Modelica.Electrical.Analog.Ideal.IdealOpAmp OpI annotation(Placement(transformation(extent = {{-14, 10}, {6, 30}})));
  Modelica.Electrical.Analog.Basic.Resistor Rd1(R = 100) annotation(Placement(transformation(extent = {{-63, 37}, {-48, 53}})));
  Modelica.Electrical.Analog.Basic.Resistor Ri(R = 10) annotation(Placement(transformation(extent = {{-37, 17}, {-22, 33}})));
  Modelica.Electrical.Analog.Basic.Resistor Rp1(R = 200) annotation(Placement(transformation(extent = {{17, 38}, {32, 54}})));
  Modelica.Electrical.Analog.Basic.Resistor Rp2(R = 50) annotation(Placement(transformation(origin = {11, 72}, extent = {{-8, -7}, {8, 7}}, rotation = 90)));
  Modelica.Electrical.Analog.Basic.Resistor Rd4(R = 100) annotation(Placement(transformation(extent = {{-55, -15}, {-40, 1}})));
  Modelica.Electrical.Analog.Sources.SignalVoltage hall2 annotation(Placement(transformation(origin = {-70, -50}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
  Modelica.Electrical.Analog.Basic.Resistor Rd3(R = 100) annotation(Placement(transformation(origin = {-70, -22}, extent = {{-8, -7}, {8, 7}}, rotation = 90)));
  Modelica.Electrical.Analog.Basic.Ground g1 annotation(Placement(transformation(extent = {{-100, -37}, {-80, -17}})));
  Modelica.Electrical.Analog.Basic.Ground g2 annotation(Placement(transformation(extent = {{-80, -91}, {-60, -71}})));
  Modelica.Electrical.Analog.Basic.Ground g3 annotation(Placement(transformation(extent = {{-34, -27}, {-14, -7}})));
  Modelica.Electrical.Analog.Sensors.CurrentSensor hall1 annotation(Placement(transformation(origin = {16, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
  Modelica.Electrical.Analog.Basic.Ground g4 annotation(Placement(transformation(extent = {{6, -84}, {26, -64}})));
  Modelica.Electrical.Analog.Basic.Ground g5 annotation(Placement(transformation(origin = {11, 93}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor phi annotation(Placement(transformation(origin = {76, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speed annotation(Placement(transformation(origin = {55, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
  Modelica.Mechanics.Rotational.Components.Inertia Jmotor(J = J) annotation(Placement(transformation(extent = {{70, -10}, {90, 10}})));
  UWBody.Examples.Systems.RobotR3.Components.AxisControlBus axisControlBus annotation(Placement(transformation(extent = {{60, -120}, {100, -80}})));
  Modelica.Blocks.Math.Gain convert1(k(unit = "V/A") = 1) annotation(Placement(transformation(extent = {{-30, -56}, {-42, -44}})));
  Modelica.Blocks.Math.Gain convert2(k(unit = "V/A") = 1) annotation(Placement(transformation(extent = {{-30, -101}, {-42, -89}})));
initial equation
  // initialize motor in steady state
  der(C.v) = 0;
  der(La.i) = 0;
equation
  connect(La.n, emf.p) annotation(Line(points = {{56, 20}, {56, 15}, {56, 10}}));
  connect(Ra.n, La.p) annotation(Line(points = {{56, 50}, {56, 40}}));
  connect(Rd2.n, diff.n1) annotation(Line(points = {{-71, 30}, {-64, 30}}));
  connect(C.n, OpI.p2) annotation(Line(points = {{6, 46}, {6, 20}}));
  connect(OpI.p2, power.p1) annotation(Line(points = {{6, 20}, {16, 20}}));
  connect(Vs.p, Rd2.p) annotation(Line(points = {{-90, 10}, {-90, 30}, {-86, 30}}));
  connect(diff.n1, Rd1.p) annotation(Line(points = {{-64, 30}, {-68, 30}, {-68, 45}, {-63, 45}}));
  connect(Rd1.n, diff.p2) annotation(Line(points = {{-48, 45}, {-44, 45}, {-44, 25}}));
  connect(diff.p2, Ri.p) annotation(Line(points = {{-44, 25}, {-37, 25}}));
  connect(Ri.n, OpI.n1) annotation(Line(points = {{-22, 25}, {-14, 25}}));
  connect(OpI.n1, C.p) annotation(Line(points = {{-14, 25}, {-14, 46}}));
  connect(power.n1, Rp1.p) annotation(Line(points = {{16, 30}, {11, 30}, {11, 46}, {17, 46}}));
  connect(power.p2, Rp1.n) annotation(Line(points = {{36, 25}, {36, 46}, {32, 46}}));
  connect(Rp1.p, Rp2.p) annotation(Line(points = {{17, 46}, {11, 46}, {11, 64}}));
  connect(power.p2, Ra.p) annotation(Line(points = {{36, 25}, {42, 25}, {42, 80}, {56, 80}, {56, 70}}));
  connect(Rd3.p, hall2.p) annotation(Line(points = {{-70, -30}, {-70, -60}}));
  connect(Rd3.n, diff.p1) annotation(Line(points = {{-70, -14}, {-70, 20}, {-64, 20}}));
  connect(Rd3.n, Rd4.p) annotation(Line(points = {{-70, -14}, {-70, -7}, {-55, -7}}));
  connect(Vs.n, g1.p) annotation(Line(points = {{-90, -10}, {-90, -17}}));
  connect(g2.p, hall2.n) annotation(Line(points = {{-70, -71}, {-70, -40}}));
  connect(Rd4.n, g3.p) annotation(Line(points = {{-40, -7}, {-24, -7}}));
  connect(g3.p, OpI.p1) annotation(Line(points = {{-24, -7}, {-24, 15}, {-14, 15}}));
  connect(g5.p, Rp2.n) annotation(Line(points = {{11, 83}, {11, 81.5}, {11, 80}}));
  connect(emf.n, hall1.p) annotation(Line(points = {{56, -10}, {56, -24}, {16, -24}, {16, -40}}));
  connect(hall1.n, g4.p) annotation(Line(points = {{16, -60}, {16, -62}, {16, -64}}));
  connect(emf.flange, phi.flange) annotation(Line(points = {{66, 0}, {66, -30}, {76, -30}}, pattern = LinePattern.Dot));
  connect(emf.flange, speed.flange) annotation(Line(points = {{66, 0}, {66, -30}, {55, -30}}, pattern = LinePattern.Dot));
  connect(OpI.n2, power.n2) annotation(Line(points = {{-4, 10}, {-4, 4}, {26, 4}, {26, 15}}));
  connect(OpI.p1, OpI.n2) annotation(Line(points = {{-14, 15}, {-14, 10}, {-4, 10}}));
  connect(OpI.p1, diff.n2) annotation(Line(points = {{-14, 15}, {-54, 15}}));
  connect(Jmotor.flange_b, flange_motor) annotation(Line(points = {{90, 0}, {100, 0}}, color = {128, 128, 128}, thickness = 0.5));
  connect(phi.phi, axisControlBus.motorAngle) annotation(Line(points = {{76, -51}, {76, -100}, {80, -100}}, color = {0, 0, 127}));
  connect(speed.w, axisControlBus.motorSpeed) annotation(Line(points = {{55, -51}, {55, -95}, {80, -95}, {80, -100}}, color = {0, 0, 127}));
  connect(hall1.i, axisControlBus.current) annotation(Line(points = {{6, -50}, {-10, -50}, {-10, -95}, {80, -95}, {80, -100}}, color = {0, 0, 127}));
  connect(hall1.i, convert1.u) annotation(Line(points = {{6, -50}, {-28.8, -50}}, color = {0, 0, 127}));
  connect(convert1.y, hall2.v) annotation(Line(points = {{-42.6, -50}, {-63, -50}}, color = {0, 0, 127}));
  connect(convert2.u, axisControlBus.current_ref) annotation(Line(points = {{-28.8, -95}, {80, -95}, {80, -100}}, color = {0, 0, 127}));
  connect(convert2.y, Vs.v) annotation(Line(points = {{-42.6, -95}, {-108, -95}, {-108, 0}, {-97, 0}}, color = {0, 0, 127}));
  connect(emf.flange, Jmotor.flange_a) annotation(Line(points = {{66, 0}, {70, 0}}));
  annotation(Documentation(info = "<html>
<p>
Default values are given for the motor of joint 1.
The input of the motor is the desired current
(the actual current is proportional to the torque
produced by the motor).
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Text(extent = {{0, 100}, {0, 60}}, textColor = {64, 64, 64}, textString = "%name", lineColor = {0, 0, 255}), Line(points = {{80, -102}, {80, -10}}, color = {255, 204, 51}, thickness = 0.5)}));
end Motor;
