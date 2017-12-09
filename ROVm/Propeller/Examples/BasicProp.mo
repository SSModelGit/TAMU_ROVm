within ROVm.Propeller.Examples;

model BasicProp
  import SI = Modelica.SIunits;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(Placement(visible = true, transformation(origin = {0, -105}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  // movement vectors
  SI.Position r_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Position vector from origin of world frame to origin of frame_b" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Absolute velocity of frame_b, resolved in world frame (= der(r_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_b resolved in world frame (= der(v_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  // parameters for motor
  parameter SI.ElectricalTorqueConstant k(start = 1) = 0.05 "Transformation coefficient" annotation(Dialog(tab = "Motor Specific"));
  parameter SI.Resistance R(start = 1) = 0.18 "Resistance of propeller motor" annotation(Dialog(tab = "Motor Specific"));
  parameter SI.Inductance L(start = 1) = 0.077 "Inductance of propeller motor" annotation(Dialog(tab = "Motor Specific"));
  parameter Real direction = 1 "Direction of motor - negative if force moves in direction opposite to torque axis" annotation(Dialog(tab = "Motor Specific"));
  // parameter for propeller mount
  parameter SI.Length r "Characteristic length of the propeller, to translate torque to force" annotation(Dialog(tab = "Propeller Body Specific"));
  parameter Modelica.Mechanics.MultiBody.Types.Axis n = {1, 0, 0} "Axis of rotation = axis of support torque (resolved in frame_a)" annotation(Evaluate = true, Dialog(tab = "Propeller Body Specific"));
  // parameters for propeller body
  parameter Boolean animation = true;
  parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter SI.Length r_CM_Propeller[3] = {0, 0, 0} "r_CM from frame_a to center of mass of propeller";
  parameter SI.Mass m_Propeller "Mass of propeller body";
  parameter SI.Density d_Propeller = 1000 "Average density of propeller body";
  parameter SI.Area A_Propeller "Overall cross sectional area effective in drag of propeller body";
  parameter SI.DimensionlessRatio c_d_Propeller = 0 "Drag coefficient of the propeller body";
  Modelica.Electrical.Analog.Basic.Resistor resistor(R = R) annotation(Placement(visible = true, transformation(origin = {-30, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Inductor inductor(L = L) annotation(Placement(visible = true, transformation(origin = {16.886, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Rotor1D propellerInertia(J = 2, n = n) "Inertia of part of propeller interacting with the water" annotation(Placement(visible = true, transformation(origin = {112.087, 93.163}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Mounting1D mounting1D(n = n, phi0 = 0) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque thrust(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_resolve) annotation(Placement(visible = true, transformation(origin = {-40, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody propeller(c_d = c_d_Propeller, A = A_Propeller, density = d_Propeller, r_CM = r_CM_Propeller, m = m_Propeller, I_11 = 0.5, I_22 = 0.5, I_33 = 0.5) "Mass of the propeller affected by torque, colocated with inertia" annotation(Placement(visible = true, transformation(origin = {53.155, -27.608}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.EMF emf1(useSupport = true, k = k * direction) annotation(Placement(visible = true, transformation(origin = {68.283, 80}, extent = {{-10, -10}, {10, 10}}, rotation = -630)));
  Modelica.Blocks.Interfaces.RealInput u annotation(Placement(visible = true, transformation(origin = {-148.397, 55}, extent = {{-20, -20}, {20, 20}}, rotation = -360), iconTransformation(origin = {0, 90}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation(Placement(visible = true, transformation(origin = {-95, 55}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(Placement(visible = true, transformation(origin = {-115, 81.742}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  thrust.force = mounting1D.housing.t * (1 / r) * direction;
  thrust.torque = zeros(3);
  r_0 = frame_b.r_0;
  v_0 = der(r_0);
  a_0 = der(v_0);
  connect(resistor.n, inductor.p) annotation(Line(visible = true, origin = {-6.557, 80}, points = {{-13.443, 0}, {13.443, 0}}, color = {10, 90, 224}));
  connect(mounting1D.frame_a, propeller.frame_a) annotation(Line(visible = true, origin = {14.385, -21.739}, points = {{-14.385, 11.739}, {-14.385, -5.869}, {28.77, -5.869}}, color = {95, 95, 95}));
  connect(propellerInertia.frame_a, mounting1D.frame_a) annotation(Line(visible = true, origin = {56.044, 11.784}, points = {{56.044, 71.379}, {56.044, -24.797}, {-56.044, -24.797}, {-56.044, -21.784}}, color = {95, 95, 95}));
  connect(thrust.frame_b, propeller.frame_a) annotation(Line(visible = true, origin = {23.36, -43.804}, points = {{-53.36, -16.196}, {16.782, -16.196}, {16.782, 16.196}, {19.795, 16.196}}, color = {95, 95, 95}));
  connect(thrust.frame_resolve, mounting1D.frame_a) annotation(Line(visible = true, origin = {-20, -21.506}, points = {{-20, -28.494}, {-20, 8.494}, {20, 8.494}, {20, 11.506}}, color = {95, 95, 95}));
  connect(propeller.frame_a, frame_b) annotation(Line(visible = true, origin = {14.385, -53.405}, points = {{28.77, 25.797}, {-14.385, 25.797}, {-14.385, -51.595}}, color = {95, 95, 95}));
  connect(emf1.support, mounting1D.flange_b) annotation(Line(visible = true, origin = {48.855, 23.333}, points = {{19.428, 46.667}, {19.428, -23.333}, {-38.855, -23.333}}));
  connect(emf1.flange, propellerInertia.flange_a) annotation(Line(visible = true, origin = {79.551, 92.109}, points = {{-11.268, -2.109}, {-11.268, 1.054}, {22.536, 1.054}}, color = {64, 64, 64}));
  connect(u, signalVoltage.v) annotation(Line(visible = true, origin = {-125.198, 55}, points = {{-23.198, 0}, {23.198, 0}}, color = {1, 37, 163}));
  connect(signalVoltage.p, resistor.p) annotation(Line(visible = true, origin = {-63.29, 57.71}, points = {{-31.71, -12.71}, {-31.71, -15.935}, {20.065, -15.935}, {20.065, 22.29}, {23.29, 22.29}}, color = {10, 90, 224}));
  connect(ground.p, signalVoltage.n) annotation(Line(visible = true, origin = {-105, 86.669}, points = {{-10, 5.073}, {-10, 8.298}, {10, 8.298}, {10, -21.669}}, color = {10, 90, 224}));
  connect(inductor.n, emf1.p) annotation(Line(visible = true, origin = {42.584, 80}, points = {{-15.698, 0}, {15.699, 0}}, color = {10, 90, 224}));
  connect(emf1.n, signalVoltage.n) annotation(Line(visible = true, origin = {10.17, 83}, points = {{68.113, -3}, {71.113, -3}, {71.113, 12}, {-105.17, 12}, {-105.17, -18}}, color = {10, 90, 224}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {50, 0.721}, fillPattern = FillPattern.Solid, points = {{50, -0.721}, {-0, 12.163}, {-50, -0.721}, {-0, -10.721}}), Polygon(visible = true, origin = {-45, -45}, fillPattern = FillPattern.Solid, points = {{45, 45}, {15, -5}, {-55, -55}, {-5, 15}}), Polygon(visible = true, origin = {-45, 45}, fillPattern = FillPattern.Solid, points = {{45, -45}, {15, 5}, {-55, 55}, {-5, -15}}), Line(visible = true, origin = {-66, -7.942}, rotation = -720, points = {{66, 80}, {66, -70}}, color = {0, 0, 255}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30)}), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end BasicProp;
