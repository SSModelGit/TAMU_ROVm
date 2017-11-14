within ROVm.Propeller.Examples;

model BasicProp
  import SI = Modelica.SIunits;
  Modelica.Electrical.Analog.Basic.EMF emf(k = k, useSupport = true) annotation(Placement(visible = true, transformation(origin = {70, 80}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation(Placement(visible = true, transformation(origin = {-148.274, 105}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(Placement(visible = true, transformation(origin = {148.315, 105}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Ideal.IdealCommutingSwitch idealCommutingSwitch annotation(Placement(visible = true, transformation(origin = {-70, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.BooleanInput u annotation(Placement(visible = true, transformation(origin = {0, 105}, extent = {{-8.969, -8.969}, {8.969, 8.969}}, rotation = -90), iconTransformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(Placement(visible = true, transformation(origin = {0, -105}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  // movement vectors
  SI.Position r_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Position vector from origin of world frame to origin of frame_b" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Absolute velocity of frame_b, resolved in world frame (= der(r_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_b resolved in world frame (= der(v_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  // parameters for motor
  parameter SI.ElectricalTorqueConstant k(start = 1) "Transformation coefficient" annotation(Dialog(tab = "Motor Specific"));
  parameter SI.Resistance R(start = 1) "Resistance of propeller motor" annotation(Dialog(tab = "Motor Specific"));
  parameter SI.Inductance L(start = 1) "Inductance of propeller motor" annotation(Dialog(tab = "Motor Specific"));
  // parameter for propeller mount
  parameter SI.Length r "Characteristic length of the propeller, to translate torque to force" annotation(Dialog(tab = "Propeller Body Specific"));
  parameter Modelica.Mechanics.MultiBody.Types.Axis n = {1, 0, 0} "Axis of rotation = axis of support torque (resolved in frame_a)" annotation(Evaluate = true, Dialog(tab = "Propeller Body Specific"));
  // parameters for propeller body
  parameter Boolean animation = true;
  parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter SI.Length r_Propeller[3] "length of overall propeller body, from frame_a to frame_b";
  parameter SI.Length r_CM_Propeller[3] "r_CM from frame_a to center of mass of propeller";
  parameter SI.Mass m_Propeller "Mass of propeller body";
  parameter SI.Density d_Propeller "Average density of propeller body";
  parameter SI.Area A_Propeller "Overall cross sectional area effective in drag of propeller body";
  parameter SI.DimensionlessRatio c_d_Propeller "Drag coefficient of the propeller body";
  Modelica.Electrical.Analog.Basic.Resistor resistor(R = R) annotation(Placement(visible = true, transformation(origin = {-30, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Inductor inductor(L = L) annotation(Placement(visible = true, transformation(origin = {30, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicPropMount basicPropMount(k = r, n = n) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Rotor1D rotor1D(J = 2) annotation(Placement(visible = true, transformation(origin = {112.087, 93.163}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  r_0 = frame_b.r_0;
  v_0 = der(r_0);
  a_0 = der(v_0);
  connect(pin_p, idealCommutingSwitch.p) annotation(Line(visible = true, origin = {-98.681, 92.5}, points = {{-49.593, 12.5}, {15.456, 12.5}, {15.456, -12.5}, {18.681, -12.5}}, color = {10, 90, 224}));
  connect(pin_n, emf.n) annotation(Line(visible = true, origin = {98.579, 92.5}, points = {{49.736, 12.5}, {-15.579, 12.5}, {-15.579, -12.5}, {-18.579, -12.5}}, color = {10, 90, 224}));
  connect(u, idealCommutingSwitch.control) annotation(Line(visible = true, origin = {-46.667, 99.333}, points = {{46.667, 5.667}, {-23.333, 5.667}, {-23.333, -11.333}}, color = {190, 52, 178}));
  connect(idealCommutingSwitch.n2, resistor.p) annotation(Line(visible = true, origin = {-50, 80}, points = {{-10, 0}, {10, 0}}, color = {10, 90, 224}));
  connect(resistor.n, inductor.p) annotation(Line(visible = true, origin = {-0, 80}, points = {{-20, 0}, {20, 0}}, color = {10, 90, 224}));
  connect(inductor.n, emf.p) annotation(Line(visible = true, origin = {50, 80}, points = {{-10, 0}, {10, 0}}, color = {10, 90, 224}));
  connect(emf.support, basicPropMount.flange_b) annotation(Line(visible = true, origin = {49.998, 23.333}, points = {{20.002, 46.667}, {20.002, -23.333}, {-40.005, -23.333}}));
  connect(basicPropMount.frame_a, frame_b) annotation(Line(visible = true, origin = {0, -57.414}, points = {{0, 47.586}, {0, -47.586}}, color = {95, 95, 95}));
  connect(rotor1D.flange_a, emf.flange) annotation(Line(visible = true, origin = {80.696, 92.109}, points = {{21.391, 1.054}, {-10.696, 1.054}, {-10.696, -2.109}}, color = {64, 64, 64}));
  connect(rotor1D.frame_a, frame_b) annotation(Line(visible = true, origin = {56.043, -48.459}, points = {{56.043, 131.622}, {56.043, -37.541}, {-56.043, -37.541}, {-56.043, -56.541}}, color = {95, 95, 95}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {50, 0.721}, fillPattern = FillPattern.Solid, points = {{50, -0.721}, {-0, 12.163}, {-50, -0.721}, {-0, -10.721}}), Polygon(visible = true, origin = {-45, -45}, fillPattern = FillPattern.Solid, points = {{45, 45}, {15, -5}, {-55, -55}, {-5, 15}}), Polygon(visible = true, origin = {-45, 45}, fillPattern = FillPattern.Solid, points = {{45, -45}, {15, 5}, {-55, 55}, {-5, -15}}), Line(visible = true, origin = {-66, -7.942}, rotation = -720, points = {{66, 80}, {66, -70}}, color = {0, 0, 255}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30)}), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end BasicProp;
