within ROVm.Examples;

model FullBlueROV2Example
  extends ROVm.Templates.BlueROV2Template(firstOrder2.k = 0.2805, firstOrder1.k = 0.3167, firstOrder4.k = 0.3904, firstOrder3.k = 0.4266, firstOrder5.k = 0.0932, firstOrder6.k = -0.0932);
  Modelica.Blocks.Sources.Constant power(k = 14.8) annotation(Placement(visible = true, transformation(origin = {-110, 150}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(power.y, firstOrder1.u) annotation(Line(visible = true, origin = {-42.171, 150}, points = {{-56.829, 0}, {56.829, 0}}, color = {1, 37, 163}));
  connect(power.y, firstOrder2.u) annotation(Line(visible = true, origin = {-83, 150}, points = {{-16, 0}, {16, 0}}, color = {1, 37, 163}));
  connect(power.y, firstOrder3.u) annotation(Line(visible = true, origin = {-2.548, 115}, points = {{-96.452, 35}, {31.151, 35}, {31.151, -35}, {34.151, -35}}, color = {1, 37, 163}));
  connect(power.y, firstOrder4.u) annotation(Line(visible = true, origin = {-76.5, 112.5}, points = {{-22.5, 37.5}, {6.5, 37.5}, {6.5, -37.5}, {9.5, -37.5}}, color = {1, 37, 163}));
  connect(power.y, firstOrder5.u) annotation(Line(visible = true, origin = {-76.5, 85}, points = {{-22.5, 65}, {6.5, 65}, {6.5, -65}, {9.5, -65}}, color = {1, 37, 163}));
  connect(power.y, firstOrder6.u) annotation(Line(visible = true, origin = {-2.548, 85}, points = {{-96.452, 65}, {31.151, 65}, {31.151, -65}, {34.151, -65}}, color = {1, 37, 163}));
  annotation(experiment(Tolerance = 1e-3), Diagram(coordinateSystem(extent = {{-220, -220}, {220, 220}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end FullBlueROV2Example;
