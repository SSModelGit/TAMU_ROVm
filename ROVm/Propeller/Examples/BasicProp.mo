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
  parameter SI.MassFlowRate k_m = 1 "Propeller shape coefficient, for representing fluid mass pushed through in some constant time t by propeller" annotation(Dialog(tab = "Propeller Body Specific"));
  parameter SI.Length k_r = 1 "Propeller shape coefficient, for rotation-to-linear conversion in torque-to-force calculation" annotation(Dialog(tab = "Propeller Body Specific"));
  parameter Real eta = 0.85 "Efficiency coefficient of the propeller, to translate torque to force" annotation(Dialog(tab = "Propeller Body Specific"));
  parameter Modelica.Mechanics.MultiBody.Types.Axis n = {1, 0, 0} "Axis of rotation = axis of support torque (resolved in frame_a)" annotation(Evaluate = true, Dialog(tab = "Propeller Body Specific"));
  // parameters for propeller body
  parameter Boolean animation = true;
  parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter SI.Length r_CM_Propeller[3] = {0, 0, 0} "r_CM from frame_a to center of mass of propeller";
  parameter SI.Mass m_Propeller "Mass of propeller body";
  parameter SI.Inertia j_Propeller = 2 "Inertia of propeller body";
  parameter SI.Density d_Propeller = 1000 "Average density of propeller body";
  parameter SI.Area A_Propeller "Overall cross sectional area effective in drag of propeller body";
  parameter SI.DimensionlessRatio c_d_Propeller = 0 "Drag coefficient of the propeller body";
  Modelica.Electrical.Analog.Basic.Resistor resistor(R = R) annotation(Placement(visible = true, transformation(origin = {-76.832, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Inductor inductor(L = L) annotation(Placement(visible = true, transformation(origin = {-45, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Rotor1D rotor(J = j_Propeller, n = n) "Inertia of part of propeller interacting with the water" annotation(Placement(visible = true, transformation(origin = {38.207, 47.849}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Mounting1D mounting1D(n = n, phi0 = 0) annotation(Placement(visible = true, transformation(origin = {-40, -0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque thrust(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_resolve) annotation(Placement(visible = true, transformation(origin = {20, -27.73}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody propeller(c_d = c_d_Propeller, A = A_Propeller, density = d_Propeller, r_CM = r_CM_Propeller, m = m_Propeller, I_11 = 0.5, I_22 = 0.5, I_33 = 0.5) "Mass of the propeller affected by torque, colocated with inertia" annotation(Placement(visible = true, transformation(origin = {-55, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.EMF emf1(useSupport = true, k = k * direction) annotation(Placement(visible = true, transformation(origin = {-1.994, 35}, extent = {{-10, -10}, {10, 10}}, rotation = -630)));
  Modelica.Blocks.Interfaces.RealInput u annotation(Placement(visible = true, transformation(origin = {-148.397, 55}, extent = {{-20, -20}, {20, 20}}, rotation = -360), iconTransformation(origin = {0, 90}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation(Placement(visible = true, transformation(origin = {-95, 55}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(Placement(visible = true, transformation(origin = {-117.51, 72.061}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.Torque loadTorque annotation(Placement(visible = true, transformation(origin = {43.23, 81.592}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Fixed fixed annotation(Placement(visible = true, transformation(origin = {97.885, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Damper damper(d = 0.1) annotation(Placement(visible = true, transformation(origin = {75, 47.849}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  thrust.force = -k_m * (k_r * rotor.w - v_0 * n) * direction * n;
  // thrust.force = -mounting1D.housing.t * (1 / k_r) * eta * direction * exp(-(Modelica.Math.Vectors.length(v_0)));
  thrust.torque = zeros(3);
  loadTorque.tau = -Modelica.Math.Vectors.length(thrust.force) * (v_0 * n) / (rotor.w + 0.001 * exp(-rotor.w ^ 2)) / eta;
  r_0 = frame_b.r_0;
  v_0 = der(r_0);
  a_0 = der(v_0);
  connect(resistor.n, inductor.p) annotation(Line(visible = true, origin = {-60.916, 35}, points = {{-5.916, -0}, {5.916, 0}}, color = {10, 90, 224}));
  connect(mounting1D.frame_a, propeller.frame_a) annotation(Line(visible = true, origin = {-41.667, -43.333}, points = {{1.667, 33.333}, {1.667, -16.667}, {-3.333, -16.667}}, color = {95, 95, 95}));
  connect(thrust.frame_resolve, mounting1D.frame_a) annotation(Line(visible = true, origin = {-10, -13.439}, points = {{30, -4.291}, {30, 0.426}, {-30, 0.426}, {-30, 3.439}}, color = {95, 95, 95}));
  connect(emf1.support, mounting1D.flange_b) annotation(Line(visible = true, origin = {-11.329, 8.333}, points = {{9.335, 16.667}, {9.335, -8.333}, {-18.671, -8.333}}));
  connect(emf1.flange, rotor.flange_a) annotation(Line(visible = true, origin = {14.922, 47.344}, points = {{-16.916, -2.344}, {-16.916, 0.668}, {10.273, 0.668}, {10.273, 0.504}, {13.285, 0.504}}, color = {64, 64, 64}));
  connect(u, signalVoltage.v) annotation(Line(visible = true, origin = {-125.198, 55}, points = {{-23.198, 0}, {23.198, 0}}, color = {1, 37, 163}));
  connect(signalVoltage.p, resistor.p) annotation(Line(visible = true, origin = {-92.277, 38.333}, points = {{-2.723, 6.667}, {-2.723, -3.333}, {5.445, -3.333}}, color = {10, 90, 224}));
  connect(ground.p, signalVoltage.n) annotation(Line(visible = true, origin = {-106.255, 78.408}, points = {{-11.255, 3.653}, {-11.255, 4.878}, {11.255, 4.878}, {11.255, -13.408}}, color = {10, 90, 224}));
  connect(inductor.n, emf1.p) annotation(Line(visible = true, origin = {-23.497, 35}, points = {{-11.503, 0}, {11.503, -0}}, color = {10, 90, 224}));
  connect(thrust.frame_b, rotor.frame_a) annotation(Line(visible = true, origin = {35.471, -5.87}, points = {{-5.471, -21.86}, {2.736, -21.86}, {2.736, 43.719}}, color = {95, 95, 95}));
  connect(loadTorque.flange, rotor.flange_b) annotation(Line(visible = true, origin = {51.555, 59.096}, points = {{1.674, 22.496}, {1.674, -11.248}, {-3.348, -11.248}}, color = {64, 64, 64}));
  connect(emf1.n, signalVoltage.n) annotation(Line(visible = true, origin = {-31.996, 54.29}, points = {{40.003, -19.29}, {43.003, -19.29}, {43.003, 13.935}, {-63.004, 13.935}, {-63.004, 10.71}}, color = {10, 90, 224}));
  connect(frame_b, propeller.frame_a) annotation(Line(visible = true, origin = {-15, -75}, points = {{15, -30}, {15, 15}, {-30, 15}}, color = {95, 95, 95}));
  connect(rotor.frame_a, propeller.frame_a) annotation(Line(visible = true, origin = {10.471, -27.384}, points = {{27.736, 65.232}, {27.736, -32.616}, {-55.471, -32.616}}, color = {95, 95, 95}));
  connect(rotor.flange_b, damper.flange_a) annotation(Line(visible = true, origin = {56.603, 47.849}, points = {{-8.396, 0}, {8.397, -0}}, color = {64, 64, 64}));
  connect(damper.flange_b, fixed.flange) annotation(Line(visible = true, origin = {89.164, 48.924}, points = {{-4.164, -1.076}, {-2.279, -1.076}, {-2.279, 1.076}, {8.721, 1.076}}, color = {64, 64, 64}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {50, 0.721}, fillPattern = FillPattern.Solid, points = {{50, -0.721}, {-0, 12.163}, {-50, -0.721}, {-0, -10.721}}), Polygon(visible = true, origin = {-45, -45}, fillPattern = FillPattern.Solid, points = {{45, 45}, {15, -5}, {-55, -55}, {-5, 15}}), Polygon(visible = true, origin = {-45, 45}, fillPattern = FillPattern.Solid, points = {{45, -45}, {15, 5}, {-55, 55}, {-5, -15}}), Line(visible = true, origin = {-66, -7.942}, rotation = -720, points = {{66, 80}, {66, -70}}, color = {0, 0, 255}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30)}), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end BasicProp;
