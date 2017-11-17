within ROVm.TestBin;

model PropellerTest
  Propeller.Examples.BasicProp basicProp(k = 1, m_Propeller = 5, A_Propeller = 0.2, n = {1, 0, 0}, R = 1, L = 1, r = 1) annotation(Placement(visible = true, transformation(origin = {-2.071, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RBodyInFluid.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-115, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-130, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V = 9) annotation(Placement(visible = true, transformation(origin = {10, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody basicBody(c_d = 0, A = 0.25, m = 5, r_CM = {1, 0, 0}, density = 1000, I_11 = 0.5, I_22 = 0.5, I_33 = 0.5) annotation(Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanStep booleanStep(startTime = 5) annotation(Placement(visible = true, transformation(origin = {-55, 55}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(Placement(visible = true, transformation(origin = {50, 25}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(constantVoltage.p, basicProp.pin_p) annotation(Line(visible = true, origin = {-8.047, 26.667}, points = {{8.047, 8.333}, {-4.024, 8.333}, {-4.024, -16.667}}, color = {10, 90, 224}));
  connect(constantVoltage.n, basicProp.pin_n) annotation(Line(visible = true, origin = {18.595, 22.5}, points = {{1.405, 12.5}, {4.63, 12.5}, {4.63, -12.5}, {-10.665, -12.5}}, color = {10, 90, 224}));
  connect(booleanStep.y, basicProp.u) annotation(Line(visible = true, origin = {-16.047, 40}, points = {{-27.953, 15}, {13.976, 15}, {13.976, -30}}, color = {190, 52, 178}));
  connect(constantVoltage.n, ground.p) annotation(Line(visible = true, origin = {35, 35}, points = {{-15, 0}, {15, 0}}, color = {10, 90, 224}));
  connect(basicBody.frame_a, basicProp.frame_b) annotation(Line(visible = true, origin = {-8.033, -23.29}, points = {{-1.967, -16.71}, {-4.979, -16.71}, {-4.979, 10.065}, {5.963, 10.065}, {5.963, 13.29}}, color = {95, 95, 95}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end PropellerTest;
