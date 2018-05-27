within ROVm.Examples;

model DataBasedBlueROV2
  extends ROVm.Templates.BlueROV2Template(firstOrder2.k = 14.8, firstOrder1.k = 14.8, firstOrder4.k = 14.8, firstOrder3.k = 14.8, firstOrder5.k = 14.8, firstOrder6.k = 14.8);
  Modelica.Blocks.Tables.CombiTable1Ds combiTable1Ds(tableName = "rcou_tab", fileName = Modelica.Utilities.Files.loadResource("modelica://ROVm.Examples.DataBasedBlueROV2/../Resources/rcou_single_table.mat"), tableOnFile = true, columns = 2:7) annotation(Placement(visible = true, transformation(origin = {-131.722, 150}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Clock clock annotation(Placement(visible = true, transformation(origin = {-175, 150}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(clock.y, combiTable1Ds.u) annotation(Line(visible = true, origin = {-153.861, 150}, points = {{-10.139, 0}, {10.139, 0}}, color = {1, 37, 163}));
  connect(combiTable1Ds.y[1], firstOrder1.u) annotation(Line(visible = true, origin = {-49.313, 157.365}, points = {{-71.409, -7.365}, {-28.267, -7.365}, {-28.267, 11.048}, {63.971, 11.048}, {63.971, -7.365}}, color = {1, 37, 163}));
  connect(combiTable1Ds.y[2], firstOrder2.u) annotation(Line(visible = true, origin = {-93.861, 150}, points = {{-26.861, 0}, {26.861, 0}}, color = {1, 37, 163}));
  connect(combiTable1Ds.y[3], firstOrder3.u) annotation(Line(visible = true, origin = {-29.677, 116.657}, points = {{-91.045, 33.343}, {29.348, 33.343}, {29.348, -33.343}, {32.348, -33.343}}, color = {1, 37, 163}));
  connect(combiTable1Ds.y[4], firstOrder4.u) annotation(Line(visible = true, origin = {-81.93, 112.5}, points = {{-38.792, 37.5}, {11.93, 37.5}, {11.93, -37.5}, {14.93, -37.5}}, color = {1, 37, 163}));
  connect(combiTable1Ds.y[5], firstOrder5.u) annotation(Line(visible = true, origin = {-81.93, 85}, points = {{-38.792, 65}, {11.93, 65}, {11.93, -65}, {14.93, -65}}, color = {1, 37, 163}));
  connect(combiTable1Ds.y[6], firstOrder6.u) annotation(Line(visible = true, origin = {-29.677, 86.657}, points = {{-91.045, 63.343}, {29.348, 63.343}, {29.348, -63.343}, {32.348, -63.343}}, color = {1, 37, 163}));
  annotation(experiment(StopTime = 500.0), Diagram(coordinateSystem(extent = {{-220, -220}, {220, 220}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end DataBasedBlueROV2;
