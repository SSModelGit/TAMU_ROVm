within ROVm.Examples;

model FullBlueROV2Example
  extends ROVm.Templates.BlueROV2Template(firstOrder2.k = 0.2967, firstOrder1.k = 0.3316, firstOrder4.k = 0.3755, firstOrder3.k = 0.4104, firstOrder5.k = 0.0897, firstOrder6.k = -0.0897);
  // battery density was 0.81
  Modelica.Blocks.Sources.Constant power(k = 14.8) annotation(Placement(visible = true, transformation(origin = {-110, 150}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(power.y, firstOrder1.u) annotation(Line(visible = true, origin = {-42.171, 150}, points = {{-56.829, 0}, {56.829, 0}}, color = {1, 37, 163}));
  connect(power.y, firstOrder2.u) annotation(Line(visible = true, origin = {-83, 150}, points = {{-16, 0}, {16, 0}}, color = {1, 37, 163}));
  connect(power.y, firstOrder3.u) annotation(Line(visible = true, origin = {-24.247, 116.657}, points = {{-74.753, 33.343}, {23.918, 33.343}, {23.918, -33.343}, {26.918, -33.343}}, color = {1, 37, 163}));
  connect(power.y, firstOrder4.u) annotation(Line(visible = true, origin = {-76.5, 112.5}, points = {{-22.5, 37.5}, {6.5, 37.5}, {6.5, -37.5}, {9.5, -37.5}}, color = {1, 37, 163}));
  connect(power.y, firstOrder5.u) annotation(Line(visible = true, origin = {-76.5, 85}, points = {{-22.5, 65}, {6.5, 65}, {6.5, -65}, {9.5, -65}}, color = {1, 37, 163}));
  connect(power.y, firstOrder6.u) annotation(Line(visible = true, origin = {-24.247, 86.657}, points = {{-74.753, 63.343}, {23.918, 63.343}, {23.918, -63.343}, {26.918, -63.343}}, color = {1, 37, 163}));
  annotation(experiment(StopTime = 750.0, __Wolfram_Algorithm = "cvodes", Tolerance = 1e-3), Diagram(coordinateSystem(extent = {{-220, -220}, {220, 220}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end FullBlueROV2Example;
