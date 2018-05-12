within ROVm.TestBin;

model PropellerTest
  Propeller.Examples.BasicProp basicProp(m_Propeller = 5, A_Propeller = 0.2, direction = -1, n = {1, 0, 0}) annotation(Placement(visible = true, transformation(origin = {-35, 45}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RBodyInFluid.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-115, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-130, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant propInput5(k = 0) annotation(Placement(visible = true, transformation(origin = {-85, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder(T = 1, k = 1) annotation(Placement(visible = true, transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody basicBody(density = 950, c_d = 1, A = 1, r_CM = {0.5, 0, 0}, m = 5) annotation(Placement(visible = true, transformation(origin = {127.851, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation5(r = {1, 0.5, 0}, animation = false) annotation(Placement(visible = true, transformation(origin = {-20, 35}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
equation
  connect(propInput5.y, firstOrder.u) annotation(Line(visible = true, origin = {-68, 70}, points = {{-6, 0}, {6, 0}}, color = {1, 37, 163}));
  connect(firstOrder.y, basicProp.u) annotation(Line(visible = true, origin = {-36.333, 64.667}, points = {{-2.667, 5.333}, {1.333, 5.333}, {1.333, -10.667}}, color = {1, 37, 163}));
  connect(fixedTranslation5.frame_b, basicProp.frame_b) annotation(Line(visible = true, origin = {-31.667, 35}, points = {{1.667, 0}, {1.667, 0}, {-3.333, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation5.frame_a, basicBody.frame_a) annotation(Line(visible = true, origin = {84.382, 27.5}, points = {{-94.382, 7.5}, {30.456, 7.5}, {30.456, -7.5}, {33.469, -7.5}}, color = {95, 95, 95}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end PropellerTest;
