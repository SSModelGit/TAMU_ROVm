within ROVm.Bin;

model spinny
  Modelica.Electrical.Analog.Basic.EMF emf annotation(Placement(visible = true, transformation(origin = {16.91, 75}, extent = {{-10, 10}, {10, -10}}, rotation = 270)));
  Modelica.Blocks.Sources.BooleanConstant booleanConstant annotation(Placement(visible = true, transformation(origin = {-135, 5}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.StepVoltage stepVoltage annotation(Placement(visible = true, transformation(origin = {-77.304, 47.28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Inertia inertia annotation(Placement(visible = true, transformation(origin = {100, 62.802}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Resistor resistor annotation(Placement(visible = true, transformation(origin = {-80, 75}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(Placement(visible = true, transformation(origin = {-40, 27.264}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Inductor inductor annotation(Placement(visible = true, transformation(origin = {-40, 75}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Ideal.IdealCommutingSwitch idealCommutingSwitch annotation(Placement(visible = true, transformation(origin = {-100, 61.709}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Modelica.Blocks.Sources.BooleanStep booleanStep(startTime = 5) annotation(Placement(visible = true, transformation(origin = {-135, 61.726}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(emf.flange, inertia.flange_a) annotation(Line(visible = true, origin = {59.559, 62.916}, points = {{-42.649, 2.084}, {-42.649, -0.928}, {27.429, -0.928}, {27.429, -0.114}, {30.441, -0.114}}, color = {64, 64, 64}));
  connect(resistor.n, inductor.p) annotation(Line(visible = true, origin = {-60, 75}, points = {{-10, 0}, {10, 0}}, color = {10, 90, 224}));
  connect(inductor.n, emf.p) annotation(Line(visible = true, origin = {-11.545, 75}, points = {{-18.455, 0}, {18.455, 0}}, color = {10, 90, 224}));
  connect(emf.n, stepVoltage.n) annotation(Line(visible = true, origin = {4.856, 61.14}, points = {{22.053, 13.86}, {25.053, 13.86}, {25.053, -13.86}, {-72.16, -13.86}}, color = {10, 90, 224}));
  connect(stepVoltage.n, ground.p) annotation(Line(visible = true, origin = {-49.101, 43.941}, points = {{-18.203, 3.339}, {9.101, 3.339}, {9.101, -6.678}}, color = {10, 90, 224}));
  connect(stepVoltage.p, idealCommutingSwitch.p) annotation(Line(visible = true, origin = {-95.768, 48.756}, points = {{8.464, -1.476}, {-4.232, -1.476}, {-4.232, 2.953}}, color = {10, 90, 224}));
  connect(idealCommutingSwitch.n2, resistor.p) annotation(Line(visible = true, origin = {-96.667, 73.903}, points = {{-3.333, -2.194}, {-3.333, 1.097}, {6.667, 1.097}}, color = {10, 90, 224}));
  connect(booleanStep.y, idealCommutingSwitch.control) annotation(Line(visible = true, origin = {-114.5, 61.717}, points = {{-9.5, 0.008}, {1.5, 0.008}, {1.5, -0.008}, {6.5, -0.008}}, color = {190, 52, 178}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end spinny;
