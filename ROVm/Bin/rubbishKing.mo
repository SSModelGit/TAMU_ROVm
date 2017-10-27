within ROVm.Bin;

model rubbishKing
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-135, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RBodyInFluid.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-135, -32.785}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBodyShape basicBody(c_d = 1, A = 0.5, r_CM = {0.01, 0, 0}, m = 5, density = 1200) annotation(Placement(visible = true, transformation(origin = {-25, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBodyShape basicBody1(c_d = 1, A = 0.5, r_CM = {0.01, 0, 0}, m = 5, density = 200) annotation(Placement(visible = true, transformation(origin = {-30, -25}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {1, 0, 0}) annotation(Placement(visible = true, transformation(origin = {-58.323, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
equation
  connect(basicBody1.frame_a, fixedTranslation.frame_a) annotation(Line(visible = true, origin = {-52.216, -20}, points = {{12.216, -5}, {-6.108, -5}, {-6.108, 10}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, basicBody.frame_a) annotation(Line(visible = true, origin = {-50.549, 16.667}, points = {{-7.774, -6.667}, {-7.774, 3.333}, {15.549, 3.333}}, color = {95, 95, 95}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end rubbishKing;
