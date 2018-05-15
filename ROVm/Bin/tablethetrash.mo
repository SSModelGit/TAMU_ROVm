within ROVm.Bin;

model tablethetrash
  Modelica.Blocks.Tables.CombiTable1Ds combiTable1Ds(tableName = "rcou_tab", fileName = Modelica.Utilities.Files.loadResource("modelica://ROVm.Bin.tablethetrash/../Resources/rcou_single_table.mat"), tableOnFile = true, columns = 2:4) annotation(Placement(visible = true, transformation(origin = {-58.361, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Clock clock annotation(Placement(visible = true, transformation(origin = {-101.639, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(clock.y, combiTable1Ds.u) annotation(Line(visible = true, origin = {-80.5, 60}, points = {{-10.139, 0}, {10.139, 0}}, color = {1, 37, 163}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end tablethetrash;
