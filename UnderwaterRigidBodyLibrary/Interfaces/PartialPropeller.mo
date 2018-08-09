within UnderwaterRigidBodyLibrary.Interfaces;

partial model PartialPropeller "Basic interface for underwater propulsion"
  import SI = Modelica.SIunits;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(Placement(visible = true, transformation(origin = {0, -105}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  // movement vectors
  Modelica.SIunits.Position r_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Position vector from origin of world frame to origin of frame_b" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  Modelica.SIunits.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Absolute velocity of frame_b, resolved in world frame (= der(r_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  Modelica.SIunits.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_b resolved in world frame (= der(v_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  // parameters for motor
  parameter Modelica.SIunits.ElectricalTorqueConstant k(start = 1) = 0.05 "Transformation coefficient" annotation(Dialog(tab = "Motor Specific"));
  parameter Modelica.SIunits.Resistance R(start = 1) = 0.18 "Resistance of propeller motor" annotation(Dialog(tab = "Motor Specific"));
  parameter Modelica.SIunits.Inductance L(start = 1) = 0.077 "Inductance of propeller motor" annotation(Dialog(tab = "Motor Specific"));
  parameter Real direction = 1 "Direction of motor - negative if force moves in direction opposite to torque axis" annotation(Dialog(tab = "Motor Specific"));
  // parameter for propeller mount
  parameter Modelica.SIunits.Mass k_m = 1 "Propeller shape coefficient, for representing fluid mass pushed through in some constant time t by propeller" annotation(Dialog(tab = "Propeller Body Specific"));
  parameter Modelica.SIunits.Length k_r = 1 "Propeller shape coefficient, for rotation-to-linear conversion in torque-to-force calculation" annotation(Dialog(tab = "Propeller Body Specific"));
  parameter Real eta = 0.85 "Efficiency coefficient of the propeller, to translate torque to force" annotation(Dialog(tab = "Propeller Body Specific"));
  parameter Real w_shift = 0.001 "Coefficient of angular velocity in power balance to prevent divide-by-zero error" annotation(Dialog(tab = "Propeller Body Specific"));
  parameter Modelica.Mechanics.MultiBody.Types.Axis n = {1, 0, 0} "Axis of rotation = axis of support torque (resolved in frame_a)" annotation(Evaluate = true, Dialog(tab = "Propeller Body Specific"));
  // parameters for propeller body
  parameter Boolean animation = true;
  parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter Modelica.SIunits.Length r_CM_Propeller[3] = {0, 0, 0} "r_CM from frame_a to center of mass of propeller";
  parameter Modelica.SIunits.Mass m_Propeller "Mass of propeller body";
  parameter Modelica.SIunits.Inertia j_Propeller = 2 "Inertia of propeller body";
  parameter Modelica.SIunits.Density d_Propeller = 1000 "Average density of propeller body";
  parameter Modelica.SIunits.Area A_Propeller "Overall cross sectional area effective in drag of propeller body";
  parameter Modelica.SIunits.DimensionlessRatio mu_d_Propeller = 0 "Drag coefficient of the propeller body";
  parameter Modelica.SIunits.RotationalDampingConstant k_d_Propeller "Drag coefficient of torque on propeller body";
  Modelica.Electrical.Analog.Basic.Resistor resistor(R = R) annotation(Placement(visible = true, transformation(origin = {-76.832, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Inductor inductor(L = L) annotation(Placement(visible = true, transformation(origin = {-45, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Rotor1D rotor(J = j_Propeller, n = n, exact = false) "Inertia of part of propeller interacting with the water" annotation(Placement(visible = true, transformation(origin = {38.207, 47.849}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Mounting1D mounting1D(n = n, phi0 = 0) annotation(Placement(visible = true, transformation(origin = {-40, -0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque thrust(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_resolve) annotation(Placement(visible = true, transformation(origin = {20, -27.73}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  UnderwaterRigidBodyLibrary.Parts.BasicBody propeller(mu_d = mu_d_Propeller, A = A_Propeller, density = d_Propeller, r_CM = r_CM_Propeller, m = m_Propeller, I_11 = 0.5, I_22 = 0.5, I_33 = 0.5, k_d = k_d_Propeller) "Mass of the propeller affected by torque, colocated with inertia" annotation(Placement(visible = true, transformation(origin = {-55, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.EMF propEMF(useSupport = true, k = k * direction) "EMF that drives the propeller" annotation(Placement(visible = true, transformation(origin = {-1.994, 35}, extent = {{-10, -10}, {10, 10}}, rotation = -630)));
  Modelica.Mechanics.Rotational.Sources.Torque loadTorque annotation(Placement(visible = true, transformation(origin = {43.23, 81.592}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Damper damper(d = 0.1) annotation(Placement(visible = true, transformation(origin = {75, 47.849}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  thrust.force = k_m * rotor.w * (k_r * rotor.w - v_0 * n) * direction * n;
  thrust.torque = zeros(3);
  // loadTorque.tau = k_m * (v_0 * n) * (k_r * rotor.w - v_0 * n) * direction * smooth(2, noEvent(if rotor.w >= 0 then 1.0 else -1.0));
  loadTorque.tau = -k_m * (v_0 * n) * (k_r * rotor.w - v_0 * n) * direction;
  // loadTorque.tau = -Modelica.Math.Vectors.length(thrust.force) * (v_0 * n) / (rotor.w + w_shift * exp(-rotor.w ^ 2)) / eta;
  // loadTorque.tau = 0;
  r_0 = frame_b.r_0;
  v_0 = der(r_0);
  a_0 = der(v_0);
  connect(resistor.n, inductor.p) annotation(Line(visible = true, origin = {-60.916, 35}, points = {{-5.916, -0}, {5.916, 0}}, color = {10, 90, 224}));
  connect(mounting1D.frame_a, propeller.frame_a) annotation(Line(visible = true, origin = {-41.667, -43.333}, points = {{1.667, 33.333}, {1.667, -16.667}, {-3.333, -16.667}}, color = {95, 95, 95}));
  connect(thrust.frame_resolve, mounting1D.frame_a) annotation(Line(visible = true, origin = {-10, -13.439}, points = {{30, -4.291}, {30, 0.426}, {-30, 0.426}, {-30, 3.439}}, color = {95, 95, 95}));
  connect(propEMF.support, mounting1D.flange_b) annotation(Line(visible = true, origin = {-11.329, 8.333}, points = {{9.335, 16.667}, {9.335, -8.333}, {-18.671, -8.333}}));
  connect(propEMF.flange, rotor.flange_a) annotation(Line(visible = true, origin = {14.922, 47.345}, points = {{-16.916, -2.345}, {-16.916, 0.668}, {10.273, 0.668}, {10.273, 0.504}, {13.285, 0.504}}, color = {64, 64, 64}));
  connect(inductor.n, propEMF.p) annotation(Line(visible = true, origin = {-23.497, 35}, points = {{-11.503, 0}, {11.503, -0}}, color = {10, 90, 224}));
  connect(thrust.frame_b, rotor.frame_a) annotation(Line(visible = true, origin = {35.471, -5.87}, points = {{-5.471, -21.86}, {2.736, -21.86}, {2.736, 43.719}}, color = {95, 95, 95}));
  connect(loadTorque.flange, rotor.flange_b) annotation(Line(visible = true, origin = {51.555, 59.096}, points = {{1.674, 22.496}, {1.674, -11.248}, {-3.348, -11.248}}, color = {64, 64, 64}));
  connect(frame_b, propeller.frame_a) annotation(Line(visible = true, origin = {-15, -75}, points = {{15, -30}, {15, 15}, {-30, 15}}, color = {95, 95, 95}));
  connect(rotor.frame_a, propeller.frame_a) annotation(Line(visible = true, origin = {10.471, -27.384}, points = {{27.736, 65.232}, {27.736, -32.616}, {-55.471, -32.616}}, color = {95, 95, 95}));
  connect(rotor.flange_b, damper.flange_a) annotation(Line(visible = true, origin = {56.603, 47.849}, points = {{-8.396, 0}, {8.397, -0}}, color = {64, 64, 64}));
  connect(mounting1D.flange_b, damper.flange_b) annotation(Line(visible = true, origin = {57.862, 23.924}, points = {{-87.862, -23.924}, {30.362, -23.924}, {30.362, 23.924}, {27.138, 23.924}}, color = {64, 64, 64}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {50, 0.721}, fillPattern = FillPattern.Solid, points = {{50, -0.721}, {0, 12.163}, {-50, -0.721}, {0, -10.721}}), Polygon(visible = true, origin = {-45, -45}, fillPattern = FillPattern.Solid, points = {{45, 45}, {15, -5}, {-55, -55}, {-5, 15}}), Polygon(visible = true, origin = {-45, 45}, fillPattern = FillPattern.Solid, points = {{45, -45}, {15, 5}, {-55, 55}, {-5, -15}}), Line(visible = true, origin = {-66, -7.942}, rotation = -720, points = {{66, 80}, {66, -70}}, color = {0, 0, 255}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Text(visible = true, extent = {{-150, 70}, {150, 110}}, textString = "%n")}), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end PartialPropeller;
