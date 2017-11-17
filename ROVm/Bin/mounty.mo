within ROVm.Bin;

model mounty
  Modelica.Electrical.Analog.Basic.EMF emf(useSupport = true) annotation(Placement(visible = true, transformation(origin = {56.886, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -360)));
  Modelica.Electrical.Analog.Ideal.IdealCommutingSwitch idealCommutingSwitch annotation(Placement(visible = true, transformation(origin = {-82.424, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Resistor resistor annotation(Placement(visible = true, transformation(origin = {-42.424, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Inductor inductor annotation(Placement(visible = true, transformation(origin = {17.231, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(Placement(visible = true, transformation(origin = {-55, -17.255}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.StepVoltage stepVoltage(V = 9) annotation(Placement(visible = true, transformation(origin = {-85, 2.071}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-130, -45}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody basicBody(density = 1000, c_d = 0, A = 0.25, r_CM = {1, 0, 0}, m = 5, I_11 = 0.5, I_22 = 0.5, I_33 = 0.5) annotation(Placement(visible = true, transformation(origin = {60, -43.443}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RBodyInFluid.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-133.212, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanStep booleanStep(startTime = 0) annotation(Placement(visible = true, transformation(origin = {-115, 55}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter Modelica.SIunits.Length b = 0.1;
  RBodyInFluid.Parts.BasicBody basicBody1(c_d = 0, A = 0.25, density = 1000, r_CM = {0, 0, 0}, m = 5, I_11 = 0.5, I_22 = 0.5, I_33 = 0.5) annotation(Placement(visible = true, transformation(origin = {52.12, -17.6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque thing(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_resolve) annotation(Placement(visible = true, transformation(origin = {-40, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Rotor1D rotor1D(J = 2) annotation(Placement(visible = true, transformation(origin = {90, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Mounting1D mounting1D annotation(Placement(visible = true, transformation(origin = {10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  if booleanStep.y then
    thing.force = Modelica.Math.Vectors.length(mounting1D.housing.t) * mounting1D.n / b;
    //thing.force = {1, 0, 0};
    //bing.force = {1, 0, 0};
    // thing.torque = cross(basicBody1.r_CM, thing.force);
    //bing.torque = zeros(3);
    //thing.torque = -emf.flange.tau * {1, 0, 0};
    thing.torque = zeros(3);
  else
    thing.force = zeros(3);
    //bing.force = zeros(3);
    thing.torque = zeros(3);
    //bing.torque = zeros(3);
  end if;
  connect(idealCommutingSwitch.n2, resistor.p) annotation(Line(visible = true, origin = {-62.424, 35}, points = {{-10, 0}, {10, 0}}, color = {10, 90, 224}));
  connect(resistor.n, inductor.p) annotation(Line(visible = true, origin = {-12.596, 35}, points = {{-19.828, 0}, {19.827, 0}}, color = {10, 90, 224}));
  connect(inductor.n, emf.p) annotation(Line(visible = true, origin = {47.001, 33.333}, points = {{-19.77, 1.667}, {9.885, 1.667}, {9.885, -3.333}}, color = {10, 90, 224}));
  connect(idealCommutingSwitch.p, stepVoltage.p) annotation(Line(visible = true, origin = {-95.968, 18.535}, points = {{3.544, 16.465}, {-2.257, 16.465}, {-2.257, -16.465}, {0.968, -16.464}}, color = {10, 90, 224}));
  connect(emf.n, stepVoltage.n) annotation(Line(visible = true, origin = {12.924, 4.714}, points = {{43.962, 5.286}, {43.962, -2.643}, {-87.924, -2.643}}, color = {10, 90, 224}));
  connect(ground.p, stepVoltage.n) annotation(Line(visible = true, origin = {-61.667, -1.038}, points = {{6.667, -6.217}, {6.667, 3.109}, {-13.333, 3.109}}, color = {10, 90, 224}));
  connect(booleanStep.y, idealCommutingSwitch.control) annotation(Line(visible = true, origin = {-89.616, 51}, points = {{-14.384, 4}, {7.192, 4}, {7.192, -8}}, color = {190, 52, 178}));
  connect(basicBody.frame_a, basicBody1.frame_a) annotation(Line(visible = true, origin = {43.584, -30.521}, points = {{6.416, -12.922}, {-2.476, -12.922}, {-2.476, 12.921}, {-1.464, 12.921}}, color = {95, 95, 95}));
  connect(thing.frame_b, basicBody1.frame_a) annotation(Line(visible = true, origin = {22.584, -38.8}, points = {{-52.584, -21.2}, {16.524, -21.2}, {16.524, 21.2}, {19.536, 21.2}}, color = {95, 95, 95}));
  connect(emf.flange, rotor1D.flange_a) annotation(Line(visible = true, origin = {73.443, 20}, points = {{-6.557, 0}, {6.557, 0}}, color = {64, 64, 64}));
  connect(emf.support, mounting1D.flange_b) annotation(Line(visible = true, origin = {33.443, 20}, points = {{13.443, 0}, {-13.443, 0}}));
  connect(mounting1D.frame_a, basicBody1.frame_a) annotation(Line(visible = true, origin = {20.707, -8.4}, points = {{-10.707, 18.4}, {-10.707, -9.2}, {21.413, -9.2}}, color = {95, 95, 95}));
  connect(rotor1D.frame_a, basicBody1.frame_a) annotation(Line(visible = true, origin = {60.067, -2.33}, points = {{29.933, 12.33}, {29.933, 9.105}, {-20.959, 9.105}, {-20.959, -15.27}, {-17.947, -15.27}}, color = {95, 95, 95}));
  connect(thing.frame_resolve, mounting1D.frame_a) annotation(Line(visible = true, origin = {-15, -6.506}, points = {{-25, -43.494}, {-25, 13.494}, {25, 13.494}, {25, 16.506}}, color = {95, 95, 95}));
  annotation(experiment(StopTime = 5, Interval = 0.001, __Wolfram_Algorithm = "rk4"), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end mounty;
