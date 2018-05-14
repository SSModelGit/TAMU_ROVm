within ROVm.Propeller.Examples;

model BasicProp
  import SI = Modelica.SIunits;
  extends UnderwaterRigidBodyLibrary.Interfaces.PartialPropeller;
  Modelica.Blocks.Interfaces.RealInput u annotation(Placement(visible = true, transformation(origin = {-148.397, 55}, extent = {{-20, -20}, {20, 20}}, rotation = -360), iconTransformation(origin = {-0, 52.352}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation(Placement(visible = true, transformation(origin = {-95, 55}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(Placement(visible = true, transformation(origin = {-117.51, 72.061}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(u, signalVoltage.v) annotation(Line(visible = true, origin = {-125.198, 55}, points = {{-23.199, 0}, {23.198, 0}}, color = {1, 37, 163}));
  connect(ground.p, signalVoltage.n) annotation(Line(visible = true, origin = {-106.255, 78.408}, points = {{-11.255, 3.653}, {-11.255, 4.878}, {11.255, 4.878}, {11.255, -13.408}}, color = {10, 90, 224}));
  connect(signalVoltage.n, propEMF.n) annotation(Line(visible = true, origin = {-31.996, 54.29}, points = {{-63.004, 10.71}, {-63.004, 13.935}, {43.002, 13.935}, {43.002, -19.29}, {40.002, -19.29}}, color = {10, 90, 224}));
  connect(signalVoltage.p, resistor.p) annotation(Line(visible = true, origin = {-92.277, 38.333}, points = {{-2.723, 6.667}, {-2.723, -3.333}, {5.445, -3.333}}, color = {10, 90, 224}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end BasicProp;
