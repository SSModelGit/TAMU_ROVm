within ROVm.Electronics.Lumens;

model Lumen
  import SI = Modelica.SIunits;
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation(Placement(visible = true, transformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  UnderwaterRigidBodyLibrary.Parts.BasicBody lumen(A = A_Lumen, animation = animation, r_CM = r_CM_Lumen, m = m_Lumen, c_d = c_d_Lumen, density = d_Lumen, sphereColor = color, k_d = k_d_Lumen) annotation(Placement(visible = true, transformation(origin = {2.301, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R = R_Lumen) annotation(Placement(visible = true, transformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // movement vectors
  SI.Position r_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Position vector from origin of world frame to origin of frame_a" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Absolute velocity of frame_a, resolved in world frame (= der(r_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_a resolved in world frame (= der(v_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  // parameters
  parameter Boolean animation = true;
  parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter SI.Length r_CM_Lumen[3] "r_CM from frame_a to center of mass of lumen";
  parameter SI.Mass m_Lumen "Mass of lumen";
  parameter SI.Density d_Lumen "Density of lumen";
  parameter SI.Area A_Lumen "Overall cross sectional area of the lumen";
  parameter SI.DimensionlessRatio c_d_Lumen "Drag coefficient of the lumen";
  parameter SI.RotationalDampingConstant k_d_Lumen = 1 "Rotational drag coefficient of the lumen";
  parameter SI.Resistance R_Lumen "Resistance of lumen";
  input Modelica.Mechanics.MultiBody.Types.Color color = Modelica.Mechanics.MultiBody.Types.Defaults.BodyColor "Color of shape" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
equation
  r_0 = frame_a.r_0;
  v_0 = der(r_0);
  a_0 = der(v_0);
  connect(lumen.frame_a, frame_a) annotation(Line(visible = true, origin = {-53.849, 0}, points = {{46.151, 0}, {-46.151, 0}}, color = {95, 95, 95}));
  connect(resistor.n, pin_n) annotation(Line(visible = true, origin = {55, 100}, points = {{-45, 0}, {45, 0}}, color = {10, 90, 224}));
  connect(resistor.p, pin_p) annotation(Line(visible = true, origin = {-55, 100}, points = {{45, 0}, {-45, 0}}, color = {10, 90, 224}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Ellipse(visible = true, lineColor = {0, 0, 127}, fillColor = {170, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 5, extent = {{-100, -100}, {100, 100}})}), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end Lumen;
