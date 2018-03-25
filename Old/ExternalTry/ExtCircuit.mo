within ExternalTry;

model ExtCircuit
  Modelica.Electrical.Analog.Basic.Ground ground1 annotation(Placement(visible = true, transformation(origin = {-20.0, -37.5}, extent = {{-7.5, -7.5}, {7.5, 7.5}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage1 annotation(Placement(visible = true, transformation(origin = {-20.0, 10.0}, extent = {{-10.0, 10.0}, {10.0, -10.0}}, rotation = 270)));
  ExternalTry.ExtBlock chirpSignal1(w_end = 1000) annotation(Placement(visible = true, transformation(origin = {-50.0, 10.0}, extent = {{-10.0, -10.0}, {10.0, 10.0}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C = 0.0002) annotation(Placement(visible = true, transformation(origin = {40.0, 10.0}, extent = {{-10.0, -10.0}, {10.0, 10.0}}, rotation = 270)));
  Modelica.Electrical.Analog.Basic.Inductor inductor1(L = 0.02) annotation(Placement(visible = true, transformation(origin = {10.0, 40.0}, extent = {{-10.0, -10.0}, {10.0, 10.0}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Resistor resistor1 annotation(Placement(visible = true, transformation(origin = {10.0, -20.0}, extent = {{10.0, -10.0}, {-10.0, 10.0}}, rotation = 1080)));
equation
  connect(signalVoltage1.n, ground1.p) annotation(Line(visible = true, origin = {-20, -15}, points = {{0, 15}, {0, -15}}, color = {10, 90, 224}));
  connect(resistor1.n, signalVoltage1.n) annotation(Line(visible = true, origin = {-13.333, -13.333}, points = {{13.333, -6.667}, {-6.667, -6.667}, {-6.667, 13.333}}, color = {10, 90, 224}));
  connect(chirpSignal1.u, signalVoltage1.v) annotation(Line(visible = true, origin = {-33.125, 10}, points = {{-6.125, 0}, {6.125, 0}}, color = {1, 37, 163}));
  connect(inductor1.n, capacitor1.p) annotation(Line(visible = true, origin = {33.333, 33.333}, points = {{-13.333, 6.667}, {6.667, 6.667}, {6.667, -13.333}}, color = {10, 90, 224}));
  connect(signalVoltage1.p, inductor1.p) annotation(Line(visible = true, origin = {-13.333, 33.333}, points = {{-6.667, -13.333}, {-6.667, 6.667}, {13.333, 6.667}}, color = {10, 90, 224}));
  connect(capacitor1.n, resistor1.p) annotation(Line(visible = true, origin = {33.333, -13.333}, points = {{6.667, 13.333}, {6.667, -6.667}, {-13.333, -6.667}}, color = {10, 90, 224}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ExtCircuit;
