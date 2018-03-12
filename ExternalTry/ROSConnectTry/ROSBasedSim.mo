within ExternalTry.ROSConnectTry;

model ROSBasedSim
  ROSBasedBlock rOSBasedBlock(samplePeriod = 0.02, portNumber = 9090) annotation(Placement(visible = true, transformation(origin = {20, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Discrete.ZeroOrderHold zeroOrderHold(samplePeriod = 0.02) annotation(Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Components.SpringDamper springDamper(d = 1, c = 1) annotation(Placement(visible = true, transformation(origin = {-65, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Components.Mass mass(L = 0, m = 1, s.fixed = true, s.start = 1, stateSelect = StateSelect.default) annotation(Placement(visible = true, transformation(origin = {-38.139, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Components.Fixed fixed(s0 = 0) annotation(Placement(visible = true, transformation(origin = {-95, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Sources.Force force annotation(Placement(visible = true, transformation(origin = {83.065, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Sensors.PositionSensor positionSensor annotation(Placement(visible = true, transformation(origin = {-11.533, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(rOSBasedBlock.y, zeroOrderHold.u) annotation(Line(visible = true, origin = {34.5, 0}, points = {{-3.5, 0}, {3.5, 0}}, color = {1, 37, 163}));
  connect(springDamper.flange_b, mass.flange_a) annotation(Line(visible = true, origin = {-51.569, 0}, points = {{-3.431, 0}, {3.431, 0}}, color = {0, 127, 0}));
  connect(fixed.flange, springDamper.flange_a) annotation(Line(visible = true, origin = {-85, 0}, points = {{-10, 0}, {10, 0}}, color = {0, 127, 0}));
  connect(zeroOrderHold.y, force.f) annotation(Line(visible = true, origin = {66.033, 0}, points = {{-5.033, 0}, {5.033, 0}}, color = {1, 37, 163}));
  connect(positionSensor.s, rOSBasedBlock.u) annotation(Line(visible = true, origin = {3.734, 0}, points = {{-4.266, 0}, {4.266, 0}}, color = {1, 37, 163}));
  connect(mass.flange_b, positionSensor.flange) annotation(Line(visible = true, origin = {-24.836, 0}, points = {{-3.303, 0}, {3.303, 0}}, color = {0, 127, 0}));
  connect(force.flange, mass.flange_b) annotation(Line(visible = true, origin = {45.784, 7.178}, points = {{47.282, -7.178}, {50.282, -7.178}, {50.282, 10.767}, {-73.922, 10.767}, {-73.922, -7.178}}, color = {0, 127, 0}));
  annotation(experiment(StopTime = 20.0, Interval = 0.001, __Wolfram_Algorithm = "explicit-euler", __Wolfram_SynchronizeWithRealTime = false), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ROSBasedSim;
