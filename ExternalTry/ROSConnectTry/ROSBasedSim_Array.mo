within ExternalTry.ROSConnectTry;

model ROSBasedSim_Array
  Modelica.Mechanics.Translational.Components.SpringDamper springDamper(d = 1, c = 1) annotation(Placement(visible = true, transformation(origin = {-65, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Components.Mass mass(L = 0, m = 1, s.fixed = true, s.start = 1, stateSelect = StateSelect.default) annotation(Placement(visible = true, transformation(origin = {-38.139, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Components.Fixed fixed(s0 = 0) annotation(Placement(visible = true, transformation(origin = {-95, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Sources.Force force annotation(Placement(visible = true, transformation(origin = {83.065, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Sensors.PositionSensor positionSensor annotation(Placement(visible = true, transformation(origin = {-11.533, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Components.SpringDamper springDamper1(d = 1, c = 1) annotation(Placement(visible = true, transformation(origin = {-64.8, -60.2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Components.Mass mass1(L = 0, m = 1, s.fixed = true, s.start = 1, stateSelect = StateSelect.default) annotation(Placement(visible = true, transformation(origin = {-37.939, -60.2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Components.Fixed fixed1(s0 = 0) annotation(Placement(visible = true, transformation(origin = {-94.8, -60.2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Sources.Force force1 annotation(Placement(visible = true, transformation(origin = {83.265, -60.2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Sensors.PositionSensor positionSensor1 annotation(Placement(visible = true, transformation(origin = {-11.333, -60.2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSBasedBlock_Array rOSBasedBlock_Array(samplePeriod = 0.05, n = 2) annotation(Placement(visible = true, transformation(origin = {20, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(springDamper.flange_b, mass.flange_a) annotation(Line(visible = true, origin = {-51.569, 0}, points = {{-3.431, 0}, {3.431, 0}}, color = {0, 127, 0}));
  connect(fixed.flange, springDamper.flange_a) annotation(Line(visible = true, origin = {-85, 0}, points = {{-10, 0}, {10, 0}}, color = {0, 127, 0}));
  connect(mass.flange_b, positionSensor.flange) annotation(Line(visible = true, origin = {-24.836, 0}, points = {{-3.303, 0}, {3.303, 0}}, color = {0, 127, 0}));
  connect(force.flange, mass.flange_b) annotation(Line(visible = true, origin = {45.784, 7.178}, points = {{47.282, -7.178}, {50.282, -7.178}, {50.282, 10.767}, {-73.922, 10.767}, {-73.922, -7.178}}, color = {0, 127, 0}));
  connect(springDamper1.flange_b, mass1.flange_a) annotation(Line(visible = true, origin = {-51.369, -60.2}, points = {{-3.431, 0}, {3.43, 0}}, color = {0, 127, 0}));
  connect(fixed1.flange, springDamper1.flange_a) annotation(Line(visible = true, origin = {-84.8, -60.2}, points = {{-10, 0}, {10, 0}}, color = {0, 127, 0}));
  connect(mass1.flange_b, positionSensor1.flange) annotation(Line(visible = true, origin = {-24.636, -60.2}, points = {{-3.303, 0}, {3.303, 0}}, color = {0, 127, 0}));
  connect(force1.flange, mass1.flange_b) annotation(Line(visible = true, origin = {33.658, -65.204}, points = {{59.607, 5.004}, {62.607, 5.004}, {62.607, -10.008}, {-61.612, -10.008}, {-61.612, 5.004}, {-61.597, 5.004}}, color = {0, 127, 0}));
  connect(positionSensor1.s, rOSBasedBlock_Array.u[2]) "Second spring position input" annotation(Line(visible = true, origin = {4.417, -45.1}, points = {{-4.75, -15.1}, {0.583, -15.1}, {0.583, 15.1}, {3.583, 15.1}}, color = {1, 37, 163}));
  connect(positionSensor.s, rOSBasedBlock_Array.u[1]) "First spring position input" annotation(Line(visible = true, origin = {4.367, -15}, points = {{-4.9, 15}, {0.633, 15}, {0.633, -15}, {3.633, -15}}, color = {1, 37, 163}));
  connect(rOSBasedBlock_Array.y[1], force.f) annotation(Line(visible = true, origin = {59.549, -15}, points = {{-28.549, -15}, {8.516, -15}, {8.516, 15}, {11.516, 15}}, color = {1, 37, 163}));
  connect(rOSBasedBlock_Array.y[2], force1.f) annotation(Line(visible = true, origin = {59.699, -45.1}, points = {{-28.699, 15.1}, {8.566, 15.1}, {8.566, -15.1}, {11.566, -15.1}}, color = {1, 37, 163}));
  annotation(experiment(StopTime = -1, Interval = 0.001, __Wolfram_Algorithm = "explicit-euler", __Wolfram_SynchronizeWithRealTime = false), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ROSBasedSim_Array;
