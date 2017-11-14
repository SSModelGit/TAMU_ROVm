within ROVm.Electronics.Battery;

model Battery
  import SI = Modelica.SIunits;
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation(Placement(visible = true, transformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody battery(A = A_Battery, animation = animation, r_CM = r_CM_Battery, m = m_Battery, c_d = c_d_Battery, density = d_Battery) annotation(Placement(visible = true, transformation(origin = {2.301, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Capacitor capacitor(v.start = V_Battery, C = C_Battery) annotation(Placement(visible = true, transformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // movement vectors
  SI.Position r_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Position vector from origin of world frame to origin of frame_a" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Absolute velocity of frame_a, resolved in world frame (= der(r_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_a resolved in world frame (= der(v_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  // parameters
  parameter Boolean animation = true;
  parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter SI.Length r_CM_Battery[3] "r_CM from frame_a to center of mass of overall battery enclosure";
  parameter SI.Mass m_Battery "Mass of overall battery enclosure";
  parameter SI.Density d_Battery "Density of battery & battery enclosure";
  parameter SI.Area A_Battery "Overall cross sectional area of the battery enclosure";
  parameter SI.DimensionlessRatio c_d_Battery "Drag coefficient of the battery enclosure";
  parameter SI.Voltage V_Battery "Starting voltage of battery" annotation(Dialog(tab = "Initialization"));
  parameter SI.Capacitance C_Battery "Capacitance of battery";
  Modelica.Electrical.Analog.Basic.Ground ground annotation(Placement(visible = true, transformation(origin = {40, 76.91}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  r_0 = frame_a.r_0;
  v_0 = der(r_0);
  a_0 = der(v_0);
  connect(battery.frame_a, frame_a) annotation(Line(visible = true, origin = {-53.849, 0}, points = {{46.151, 0}, {-46.151, 0}}, color = {95, 95, 95}));
  connect(capacitor.p, pin_p) annotation(Line(visible = true, origin = {-55, 100}, points = {{45, 0}, {-45, 0}}, color = {10, 90, 224}));
  connect(capacitor.n, pin_n) annotation(Line(visible = true, origin = {55, 100}, points = {{-45, 0}, {45, 0}}, color = {10, 90, 224}));
  connect(capacitor.n, ground.p) annotation(Line(visible = true, origin = {30, 95.637}, points = {{-20, 4.363}, {10, 4.363}, {10, -8.727}}, color = {10, 90, 224}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, fillColor = {250, 238, 57}, pattern = LinePattern.DashDotDot, fillPattern = FillPattern.Backward, lineThickness = 5, extent = {{-90, -90}, {90, 90}})}));
end Battery;
