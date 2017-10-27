within UWmBody.UWExamples.Systems.RobotR3;

model oneAxis "Model of one axis of robot (controller, motor, gearbox) with simple load"
  extends Modelica.Icons.Example;
  parameter SI.Mass mLoad(min = 0) = 15 "Mass of load";
  parameter Real kp = 5 "Gain of position controller of axis 2";
  parameter Real ks = 0.5 "Gain of speed controller of axis 2";
  parameter SI.Time Ts = 0.05 "Time constant of integrator of speed controller of axis 2";
  parameter Real startAngle(unit = "deg") = 0 "Start angle of axis 2";
  parameter Real endAngle(unit = "deg") = 120 "End angle of axis 2";
  parameter SI.Time swingTime = 0.5 "Additional time after reference motion is in rest before simulation is stopped";
  parameter SI.AngularVelocity refSpeedMax = 3 "Maximum reference speed";
  parameter SI.AngularAcceleration refAccMax = 10 "Maximum reference acceleration";
  Components.AxisType1 axis(w = 5500, ratio = 210, c = 8, cd = 0.01, Rv0 = 0.5, Rv1 = 0.1 / 130, kp = kp, ks = ks, Ts = Ts) annotation(Placement(transformation(extent = {{20, 0}, {40, 20}})));
  Modelica.Mechanics.Rotational.Components.Inertia load(J = 1.3 * mLoad) annotation(Placement(transformation(extent = {{54, 0}, {74, 20}})));
  Components.PathPlanning1 pathPlanning(swingTime = swingTime, angleBegDeg = startAngle, angleEndDeg = endAngle, speedMax = refSpeedMax, accMax = refAccMax) annotation(Placement(transformation(extent = {{-60, 0}, {-40, 20}})));
protected
  Components.ControlBus controlBus annotation(Placement(transformation(extent = {{-32, 10}, {8, 50}}, origin = {2, -0}, rotation = 0), visible = true));
equation
  connect(axis.flange, load.flange_a) annotation(Line(points = {{40, 10}, {54, 10}}, color = {64, 64, 64}, thickness = 0.5, visible = true));
  connect(pathPlanning.controlBus, controlBus) annotation(Line(points = {{-40, 10}, {-15, 10}, {-15, 28}, {-12, 28}, {-10, 30}}, color = {255, 204, 51}, thickness = 0.5, visible = true));
  connect(controlBus.axisControlBus1, axis.axisControlBus) annotation(Text(string = "%first", index = -1, extent = [-6, 3; -6, 3]), Line(points = {{-10, 30}, {-12, 29}, {-9, 29}, {-9, 10}, {20, 10}}, color = {255, 204, 51}, thickness = 0.5, visible = true));
  annotation(Documentation(info = "<html>
<p>
With this model one axis of the r3 robot is checked.
The mechanical structure is replaced by a simple
load inertia.
</p>
</html>"), experiment(StopTime = 1.6), __Dymola_Commands(file = "modelica://Modelica/Resources/Scripts/Dymola/Mechanics/MultiBody/Examples/Systems/oneAxisPlot.mos" "Plot result"));
end oneAxis;
