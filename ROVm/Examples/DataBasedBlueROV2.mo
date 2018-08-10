within ROVm.Examples;

model DataBasedBlueROV2
  extends ROVm.Templates.BlueROV2Template(redeclare replaceable Modelica.Blocks.Continuous.FirstOrder firstOrder2(T = 0.1), redeclare replaceable Modelica.Blocks.Continuous.FirstOrder firstOrder1(T = 0.1), redeclare replaceable Modelica.Blocks.Continuous.FirstOrder firstOrder3(T = 0.1), redeclare replaceable Modelica.Blocks.Continuous.FirstOrder firstOrder4(T = 0.1), redeclare replaceable Modelica.Blocks.Continuous.FirstOrder firstOrder6(T = 0.1), redeclare replaceable Modelica.Blocks.Continuous.FirstOrder firstOrder5(T = 0.1), propParams.k_m = 1);
  Modelica.Blocks.Tables.CombiTable1Ds combiTable1Ds(tableName = "inout_mat_parsed", tableOnFile = true, columns = 2:13, fileName = Modelica.Utilities.Files.loadResource("modelica://ROVm.Examples.DataBasedBlueROV2/../../Resources/IN_OUT_LOG15.mat")) annotation(Placement(visible = true, transformation(origin = {-131.722, 150}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Clock clock annotation(Placement(visible = true, transformation(origin = {-175, 150}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity annotation(Placement(visible = true, transformation(origin = {170, 45}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity annotation(Placement(visible = true, transformation(origin = {170, -46.751}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.SIunits.Acceleration lin_a[3];
  Modelica.SIunits.AngularAcceleration ang_a[3];
equation
  lin_a = der(absoluteVelocity.v);
  ang_a = der(absoluteAngularVelocity.w);
  connect(clock.y, combiTable1Ds.u) annotation(Line(visible = true, origin = {-153.861, 150}, points = {{-10.139, 0}, {10.139, 0}}, color = {1, 37, 163}));
  connect(combiTable1Ds.y[1], firstOrder1.u) annotation(Line(visible = true, origin = {-49.313, 157.365}, points = {{-71.409, -7.365}, {-28.267, -7.365}, {-28.267, 11.048}, {63.971, 11.048}, {63.971, -7.365}}, color = {1, 37, 163}));
  connect(combiTable1Ds.y[2], firstOrder2.u) annotation(Line(visible = true, origin = {-93.861, 150}, points = {{-26.861, 0}, {26.861, 0}}, color = {1, 37, 163}));
  connect(combiTable1Ds.y[3], firstOrder3.u) annotation(Line(visible = true, origin = {-29.677, 116.656}, points = {{-91.045, 33.344}, {29.348, 33.344}, {29.348, -33.343}, {32.348, -33.343}}, color = {1, 37, 163}));
  connect(combiTable1Ds.y[4], firstOrder4.u) annotation(Line(visible = true, origin = {-81.93, 112.5}, points = {{-38.792, 37.5}, {11.93, 37.5}, {11.93, -37.5}, {14.93, -37.5}}, color = {1, 37, 163}));
  connect(combiTable1Ds.y[5], firstOrder5.u) annotation(Line(visible = true, origin = {-81.93, 85}, points = {{-38.792, 65}, {11.93, 65}, {11.93, -65}, {14.93, -65}}, color = {1, 37, 163}));
  connect(combiTable1Ds.y[6], firstOrder6.u) annotation(Line(visible = true, origin = {-29.677, 86.656}, points = {{-91.045, 63.344}, {29.348, 63.344}, {29.348, -63.343}, {32.348, -63.343}}, color = {1, 37, 163}));
  connect(rov_origin.frame_b, absoluteVelocity.frame_a) annotation(Line(visible = true, origin = {153.5, 32.5}, points = {{-13.5, -12.5}, {3.5, -12.5}, {3.5, 12.5}, {6.5, 12.5}}, color = {95, 95, 95}));
  connect(rov_COM.frame_b, absoluteAngularVelocity.frame_a) annotation(Line(visible = true, origin = {153.5, -35.875}, points = {{-13.5, 10.875}, {3.5, 10.875}, {3.5, -10.875}, {6.5, -10.875}}, color = {95, 95, 95}));
  // Logs 2, 3, 9, 10, 11, 12, 17 are immutable - nothing to remove
  // Log 7, 13 is left unchanged - nothing to remove, simulates fine
  // Log 8 has been parsed - does not simulate
  // Logs 1, 4, 5, 6, 15, 16 have been parsed - simulates fine
  annotation(experiment(StopTime = 1400.0, __Wolfram_Algorithm = "dassl"), Diagram(coordinateSystem(extent = {{-220, -220}, {220, 220}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end DataBasedBlueROV2;
