within UWBody.Examples.Systems.RobotR3;

model fullRobot "6 degree of freedom robot with path planning, controllers, motors, brakes, gears and mechanics"
  extends Modelica.Icons.Example;
  parameter SI.Mass mLoad(min = 0) = 15 "Mass of load";
  parameter SI.Position rLoad[3] = {0.1, 0.25, 0.1} "Distance from last flange to load mass";
  parameter Modelica.SIunits.Acceleration g = 9.81 "Gravity acceleration";
  parameter SI.Time refStartTime = 0 "Start time of reference motion";
  parameter SI.Time refSwingTime = 0.5 "Additional time after reference motion is in rest before simulation is stopped";
  parameter Real startAngle1(unit = "deg") = -60 "Start angle of axis 1" annotation(Dialog(tab = "Reference", group = "startAngles"));
  parameter Real startAngle2(unit = "deg") = 20 "Start angle of axis 2" annotation(Dialog(tab = "Reference", group = "startAngles"));
  parameter Real startAngle3(unit = "deg") = 90 "Start angle of axis 3" annotation(Dialog(tab = "Reference", group = "startAngles"));
  parameter Real startAngle4(unit = "deg") = 0 "Start angle of axis 4" annotation(Dialog(tab = "Reference", group = "startAngles"));
  parameter Real startAngle5(unit = "deg") = -110 "Start angle of axis 5" annotation(Dialog(tab = "Reference", group = "startAngles"));
  parameter Real startAngle6(unit = "deg") = 0 "Start angle of axis 6" annotation(Dialog(tab = "Reference", group = "startAngles"));
  parameter Real endAngle1(unit = "deg") = 60 "End angle of axis 1" annotation(Dialog(tab = "Reference", group = "endAngles"));
  parameter Real endAngle2(unit = "deg") = -70 "End angle of axis 2" annotation(Dialog(tab = "Reference", group = "endAngles"));
  parameter Real endAngle3(unit = "deg") = -35 "End angle of axis 3" annotation(Dialog(tab = "Reference", group = "endAngles"));
  parameter Real endAngle4(unit = "deg") = 45 "End angle of axis 4" annotation(Dialog(tab = "Reference", group = "endAngles"));
  parameter Real endAngle5(unit = "deg") = 110 "End angle of axis 5" annotation(Dialog(tab = "Reference", group = "endAngles"));
  parameter Real endAngle6(unit = "deg") = 45 "End angle of axis 6" annotation(Dialog(tab = "Reference", group = "endAngles"));
  parameter SI.AngularVelocity refSpeedMax[6] = {3, 1.5, 5, 3.1, 3.1, 4.1} "Maximum reference speeds of all joints" annotation(Dialog(tab = "Reference", group = "Limits"));
  parameter SI.AngularAcceleration refAccMax[6] = {15, 15, 15, 60, 60, 60} "Maximum reference accelerations of all joints" annotation(Dialog(tab = "Reference", group = "Limits"));
  parameter Real kp1 = 5 "Gain of position controller" annotation(Dialog(tab = "Controller", group = "Axis 1"));
  parameter Real ks1 = 0.5 "Gain of speed controller" annotation(Dialog(tab = "Controller", group = "Axis 1"));
  parameter SI.Time Ts1 = 0.05 "Time constant of integrator of speed controller" annotation(Dialog(tab = "Controller", group = "Axis 1"));
  parameter Real kp2 = 5 "Gain of position controller" annotation(Dialog(tab = "Controller", group = "Axis 2"));
  parameter Real ks2 = 0.5 "Gain of speed controller" annotation(Dialog(tab = "Controller", group = "Axis 2"));
  parameter SI.Time Ts2 = 0.05 "Time constant of integrator of speed controller" annotation(Dialog(tab = "Controller", group = "Axis 2"));
  parameter Real kp3 = 5 "Gain of position controller" annotation(Dialog(tab = "Controller", group = "Axis 3"));
  parameter Real ks3 = 0.5 "Gain of speed controller" annotation(Dialog(tab = "Controller", group = "Axis 3"));
  parameter SI.Time Ts3 = 0.05 "Time constant of integrator of speed controller" annotation(Dialog(tab = "Controller", group = "Axis 3"));
  parameter Real kp4 = 5 "Gain of position controller" annotation(Dialog(tab = "Controller", group = "Axis 4"));
  parameter Real ks4 = 0.5 "Gain of speed controller" annotation(Dialog(tab = "Controller", group = "Axis 4"));
  parameter SI.Time Ts4 = 0.05 "Time constant of integrator of speed controller" annotation(Dialog(tab = "Controller", group = "Axis 4"));
  parameter Real kp5 = 5 "Gain of position controller" annotation(Dialog(tab = "Controller", group = "Axis 5"));
  parameter Real ks5 = 0.5 "Gain of speed controller" annotation(Dialog(tab = "Controller", group = "Axis 5"));
  parameter SI.Time Ts5 = 0.05 "Time constant of integrator of speed controller" annotation(Dialog(tab = "Controller", group = "Axis 5"));
  parameter Real kp6 = 5 "Gain of position controller" annotation(Dialog(tab = "Controller", group = "Axis 6"));
  parameter Real ks6 = 0.5 "Gain of speed controller" annotation(Dialog(tab = "Controller", group = "Axis 6"));
  parameter SI.Time Ts6 = 0.05 "Time constant of integrator of speed controller" annotation(Dialog(tab = "Controller", group = "Axis 6"));
  Components.MechanicalStructure mechanics(mLoad = mLoad, rLoad = rLoad, g = g) annotation(Placement(transformation(extent = {{35.583, -35.583}, {96.583, 25.417}}, origin = {-0.583, -1.417}, rotation = 0), visible = true));
  UWBody.Examples.Systems.RobotR3.Components.PathPlanning6 pathPlanning(naxis = 6, angleBegDeg = {startAngle1, startAngle2, startAngle3, startAngle4, startAngle5, startAngle6}, angleEndDeg = {endAngle1, endAngle2, endAngle3, endAngle4, endAngle5, endAngle6}, speedMax = refSpeedMax, accMax = refAccMax, startTime = refStartTime, swingTime = refSwingTime) annotation(Placement(transformation(extent = {{-5, 50}, {-25, 70}})));
  RobotR3.Components.AxisType1 axis1(w = 4590, ratio = -105, c = 43, cd = 0.005, Rv0 = 0.4, Rv1 = 0.13 / 160, kp = kp1, ks = ks1, Ts = Ts1) annotation(Placement(transformation(extent = {{-25, -75}, {-5, -55}})));
  RobotR3.Components.AxisType1 axis2(w = 5500, ratio = 210, c = 8, cd = 0.01, Rv1 = 0.1 / 130, Rv0 = 0.5, kp = kp2, ks = ks2, Ts = Ts2) annotation(Placement(transformation(extent = {{-25, -55}, {-5, -35}})));
  RobotR3.Components.AxisType1 axis3(w = 5500, ratio = 60, c = 58, cd = 0.04, Rv0 = 0.7, Rv1 = 0.2 / 130, kp = kp3, ks = ks3, Ts = Ts3) annotation(Placement(transformation(extent = {{-25, -35}, {-5, -15}})));
  RobotR3.Components.AxisType2 axis4(k = 0.2365, w = 6250, D = 0.55, J = 1.6e-4, ratio = -99, Rv0 = 21.8, Rv1 = 9.8, peak = 26.7 / 21.8, kp = kp4, ks = ks4, Ts = Ts4) annotation(Placement(transformation(extent = {{-25, -15}, {-5, 5}})));
  RobotR3.Components.AxisType2 axis5(k = 0.2608, w = 6250, D = 0.55, J = 1.8e-4, ratio = 79.2, Rv0 = 30.1, Rv1 = 0.03, peak = 39.6 / 30.1, kp = kp5, ks = ks5, Ts = Ts5) annotation(Placement(transformation(extent = {{-25, 5}, {-5, 25}})));
  RobotR3.Components.AxisType2 axis6(k = 0.0842, w = 7400, D = 0.27, J = 4.3e-5, ratio = -99, Rv0 = 10.9, Rv1 = 3.92, peak = 16.8 / 10.9, kp = kp6, ks = ks6, Ts = Ts6) annotation(Placement(transformation(extent = {{-25, 25}, {-5, 45}})));
protected
  Components.ControlBus controlBus annotation(Placement(transformation(origin = {-80, -10}, extent = {{-20, -20}, {20, 20}}, rotation = 90), visible = true));
equation
  connect(axis2.flange, mechanics.axis2) annotation(Line(points = {{-5, -45}, {20, -45}, {20, -23.275}, {33.475, -23.275}}, visible = true, color = {64, 64, 64}));
  connect(axis1.flange, mechanics.axis1) annotation(Line(points = {{-5, -65}, {25, -65}, {25, -32.425}, {33.475, -32.425}}, visible = true, color = {64, 64, 64}));
  connect(axis3.flange, mechanics.axis3) annotation(Line(points = {{-5, -25}, {15, -25}, {15, -14.125}, {33.475, -14.125}}, visible = true, color = {64, 64, 64}));
  connect(axis4.flange, mechanics.axis4) annotation(Line(points = {{-5, -5}, {15, -5}, {33.475, -4.975}}, visible = true, color = {64, 64, 64}));
  connect(axis5.flange, mechanics.axis5) annotation(Line(points = {{-10, 15}, {10, 15}, {10, 4.175}, {28.475, 4.175}}, visible = true, origin = {5, 0}, color = {64, 64, 64}));
  connect(axis6.flange, mechanics.axis6) annotation(Line(points = {{-5, 35}, {25, 35}, {25, 13.325}, {33.475, 13.325}}, visible = true, color = {64, 64, 64}));
  connect(controlBus, pathPlanning.controlBus) annotation(Line(points = {{-18.333, -46.667}, {-18.333, 23.333}, {36.667, 23.333}}, color = {255, 204, 51}, thickness = 0.5, visible = true, origin = {-61.667, 36.667}));
  connect(controlBus.axisControlBus1, axis1.axisControlBus) annotation(Text(string = "%first", index = -1, extent = [-6, 3; -6, 3]), Line(points = {{-80, -10}, {-80.1, -14.5}, {-80, -17.5}, {-65, -17.5}, {-65, -65}, {-25, -65}}, color = {255, 204, 51}, thickness = 0.5, visible = true));
  connect(controlBus.axisControlBus2, axis2.axisControlBus) annotation(Text(string = "%first", index = -1, extent = [-6, 3; -6, 3]), Line(points = {{-80, -10}, {-80, -10}, {-80, -15}, {-62.5, -15}, {-62.5, -45}, {-25, -45}}, color = {255, 204, 51}, thickness = 0.5, visible = true));
  connect(controlBus.axisControlBus3, axis3.axisControlBus) annotation(Text(string = "%first", index = -1, extent = [-6, 3; -6, 3]), Line(points = {{-80, -10}, {-77, -9.9}, {-77, -12.5}, {-60, -12.5}, {-60, -25}, {-25, -25}}, color = {255, 204, 51}, thickness = 0.5, visible = true));
  connect(controlBus.axisControlBus4, axis4.axisControlBus) annotation(Text(string = "%first", index = -1, extent = [-6, 3; -6, 3]), Line(points = {{-80, -10}, {-60, -10}, {-60, -5}, {-25, -5}}, color = {255, 204, 51}, thickness = 0.5, visible = true));
  connect(controlBus.axisControlBus5, axis5.axisControlBus) annotation(Text(string = "%first", index = -1, extent = [-6, 3; -6, 3]), Line(points = {{-80, -10}, {-77.5, -10}, {-77.5, -7.5}, {-62.5, -7.5}, {-62.5, 15}, {-25, 15}}, color = {255, 204, 51}, thickness = 0.5, visible = true));
  connect(controlBus.axisControlBus6, axis6.axisControlBus) annotation(Text(string = "%first", index = -1, extent = [-6, 3; -6, 3]), Line(points = {{-80, -10}, {-80, -10}, {-80, -5}, {-65, -5}, {-65, 35}, {-25, 35}}, color = {255, 204, 51}, thickness = 0.5, visible = true));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})), experiment(StopTime = 2), __Dymola_Commands(file = "modelica://Modelica/Resources/Scripts/Dymola/Mechanics/MultiBody/Examples/Systems/Run.mos" "Simulate", file = "modelica://Modelica/Resources/Scripts/Dymola/Mechanics/MultiBody/Examples/Systems/fullRobotPlot.mos" "Plot result of axis 3 + animate"), Documentation(info = "<html>
<p>
This is a detailed model of the robot. For animation CAD data
is used. Translate and simulate with the default settings
(default simulation time = 3 s). Use command script \"modelica://Modelica/Resources/Scripts/Dymola/Mechanics/MultiBody/Examples/Systems/fullRobotPlot.mos\"
to plot variables.
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Systems/r3_fullRobot.png\" ALT=\"model Examples.Loops.Systems.RobotR3.fullRobot\">
</p>
</html>"));
end fullRobot;
