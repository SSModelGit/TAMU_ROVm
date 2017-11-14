within ROVm.Bin;

model mounty
  Modelica.Mechanics.MultiBody.Parts.Mounting1D mounting1D annotation(Placement(visible = true, transformation(origin = {-13.106, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Rotor1D rotor1D(J = 2) annotation(Placement(visible = true, transformation(origin = {90, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.EMF emf(useSupport = true) annotation(Placement(visible = true, transformation(origin = {56.886, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -360)));
  Modelica.Electrical.Analog.Ideal.IdealCommutingSwitch idealCommutingSwitch annotation(Placement(visible = true, transformation(origin = {-82.424, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Resistor resistor annotation(Placement(visible = true, transformation(origin = {-42.424, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Inductor inductor annotation(Placement(visible = true, transformation(origin = {17.231, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(Placement(visible = true, transformation(origin = {-55, -17.255}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.StepVoltage stepVoltage(V = 9) annotation(Placement(visible = true, transformation(origin = {-85, 2.071}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-130, -45}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody basicBody(density = 1000, c_d = 0, A = 1, r_CM = {0, 0, 1}, m = 5) annotation(Placement(visible = true, transformation(origin = {-10, -31.75}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RBodyInFluid.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-133.212, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanStep booleanStep(startTime = 5) annotation(Placement(visible = true, transformation(origin = {-115, 55}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter Modelica.SIunits.Length b = 1;
equation
  //  force.force = cutTorque.torque / b;
  connect(idealCommutingSwitch.n2, resistor.p) annotation(Line(visible = true, origin = {-62.424, 35}, points = {{-10, 0}, {10, 0}}, color = {10, 90, 224}));
  connect(resistor.n, inductor.p) annotation(Line(visible = true, origin = {-12.596, 35}, points = {{-19.828, 0}, {19.827, 0}}, color = {10, 90, 224}));
  connect(inductor.n, emf.p) annotation(Line(visible = true, origin = {47.001, 33.333}, points = {{-19.77, 1.667}, {9.885, 1.667}, {9.885, -3.333}}, color = {10, 90, 224}));
  connect(emf.support, mounting1D.flange_b) annotation(Line(visible = true, origin = {21.89, 20}, points = {{24.996, 0}, {-24.996, 0}}));
  connect(emf.flange, rotor1D.flange_a) annotation(Line(visible = true, origin = {73.443, 20}, points = {{-6.557, 0}, {6.557, 0}}, color = {64, 64, 64}));
  connect(idealCommutingSwitch.p, stepVoltage.p) annotation(Line(visible = true, origin = {-95.968, 18.535}, points = {{3.544, 16.465}, {-2.257, 16.465}, {-2.257, -16.465}, {0.968, -16.464}}, color = {10, 90, 224}));
  connect(emf.n, stepVoltage.n) annotation(Line(visible = true, origin = {12.924, 4.714}, points = {{43.962, 5.286}, {43.962, -2.643}, {-87.924, -2.643}}, color = {10, 90, 224}));
  connect(ground.p, stepVoltage.n) annotation(Line(visible = true, origin = {-61.667, -1.038}, points = {{6.667, -6.217}, {6.667, 3.109}, {-13.333, 3.109}}, color = {10, 90, 224}));
  connect(basicBody.frame_a, mounting1D.frame_a) annotation(Line(visible = true, origin = {-18.447, -7.905}, points = {{-1.553, -23.845}, {-4.565, -23.845}, {-4.565, 14.892}, {5.341, 14.892}, {5.341, 17.905}}, color = {95, 95, 95}));
  connect(rotor1D.frame_a, basicBody.frame_a) annotation(Line(visible = true, origin = {22.795, -7.99}, points = {{67.205, 17.99}, {67.205, 14.765}, {-45.808, 14.765}, {-45.808, -23.76}, {-42.795, -23.76}}, color = {95, 95, 95}));
  connect(booleanStep.y, idealCommutingSwitch.control) annotation(Line(visible = true, origin = {-89.616, 51}, points = {{-14.384, 4}, {7.192, 4}, {7.192, -8}}, color = {190, 52, 178}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end mounty;
