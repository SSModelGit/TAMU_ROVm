within ROVm.Examples;

model InputBasedBlueROV2
  extends ROVm.Templates.BlueROV2Template(redeclare replaceable Modelica.Blocks.Continuous.FirstOrder firstOrder2(T = 0.1, k = voltageSource), redeclare replaceable Modelica.Blocks.Continuous.FirstOrder firstOrder1(T = 0.1, k = voltageSource), redeclare replaceable Modelica.Blocks.Continuous.FirstOrder firstOrder3(T = 0.1, k = voltageSource), redeclare replaceable Modelica.Blocks.Continuous.FirstOrder firstOrder4(T = 0.1, k = voltageSource), redeclare replaceable Modelica.Blocks.Continuous.FirstOrder firstOrder6(T = 0.1, k = voltageSource), redeclare replaceable Modelica.Blocks.Continuous.FirstOrder firstOrder5(T = 0.1, k = voltageSource), basicProp2.direction_b = 1, basicProp4.direction_b = 1, basicProp5.direction_b = 1);
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity annotation(Placement(visible = true, transformation(origin = {170, 45}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity annotation(Placement(visible = true, transformation(origin = {170, -46.751}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.SIunits.Acceleration lin_a[3];
  Modelica.SIunits.AngularAcceleration ang_a[3];
  parameter Modelica.SIunits.Voltage voltageSource = 1 "Battery supply voltage";
  Modelica.Blocks.Math.MatrixGain matrixGain(K = 14.8 * identity(6)) annotation(Placement(visible = true, transformation(origin = {-93.133, 150}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput u[6] annotation(Placement(visible = true, transformation(origin = {-150, 150}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-95.342, 82.927}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  lin_a = der(absoluteVelocity.v);
  ang_a = der(absoluteAngularVelocity.w);
  connect(matrixGain.y[1], firstOrder1.u) annotation(Line(visible = true, origin = {-40.563, 157.144}, points = {{-41.57, -7.144}, {-34.437, -7.144}, {-34.437, 10.717}, {55.221, 10.717}, {55.221, -7.144}}, color = {1, 37, 163}));
  connect(matrixGain.y[2], firstOrder2.u) annotation(Line(visible = true, origin = {-74.567, 150}, points = {{-7.567, 0}, {7.567, 0}}, color = {1, 37, 163}));
  connect(matrixGain.y[3], firstOrder3.u) annotation(Line(visible = true, origin = {-45.358, 143.807}, points = {{-36.775, 6.193}, {-29.642, 6.193}, {-29.642, 24.054}, {48.029, 24.054}, {48.029, -60.494}}, color = {1, 37, 163}));
  connect(matrixGain.y[4], firstOrder4.u) annotation(Line(visible = true, origin = {-72.283, 112.5}, points = {{-9.85, 37.5}, {2.283, 37.5}, {2.283, -37.5}, {5.283, -37.5}}, color = {1, 37, 163}));
  connect(matrixGain.y[5], firstOrder5.u) annotation(Line(visible = true, origin = {-72.283, 85}, points = {{-9.85, 65}, {2.283, 65}, {2.283, -65}, {5.283, -65}}, color = {1, 37, 163}));
  connect(matrixGain.y[6], firstOrder6.u) annotation(Line(visible = true, origin = {-39.103, 113.725}, points = {{-43.031, 36.275}, {-35.897, 36.275}, {-35.897, 54.136}, {36.526, 54.136}, {36.526, -90.412}, {41.774, -90.412}}, color = {1, 37, 163}));
  connect(rov_origin.frame_b, absoluteVelocity.frame_a) annotation(Line(visible = true, origin = {153.5, 32.5}, points = {{-13.5, -12.5}, {3.5, -12.5}, {3.5, 12.5}, {6.5, 12.5}}, color = {95, 95, 95}));
  connect(rov_COM.frame_b, absoluteAngularVelocity.frame_a) annotation(Line(visible = true, origin = {153.5, -35.875}, points = {{-13.5, 10.875}, {3.5, 10.875}, {3.5, -10.875}, {6.5, -10.875}}, color = {95, 95, 95}));
  // Logs 2, 3, 9, 10, 11, 12, 17 are immutable - nothing to remove
  // Log 7, 13 is left unchanged - nothing to remove, simulates fine
  // Log 8 has been parsed - does not simulate
  // Logs 1, 4, 5, 6, 15, 16 have been parsed - simulates fine
  connect(u, matrixGain.u) annotation(Line(visible = true, origin = {-127.566, 150}, points = {{-22.434, 0}, {22.433, 0}}, color = {1, 37, 163}));
  annotation(experiment(StartTime = 0, StopTime = 265, __Wolfram_Algorithm = "dassl"), Diagram(coordinateSystem(extent = {{-220, -220}, {220, 220}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end InputBasedBlueROV2;
