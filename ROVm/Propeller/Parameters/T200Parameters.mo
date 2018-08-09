within ROVm.Propeller.Parameters;

record T200Parameters "Record of the propeller's parameters"
  import SI = Modelica.SIunits;
  // parameters are currently based on T200 propeller used on the BlueROV2 - specs found at http://docs.bluerobotics.com/thrusters/t200/#t200-thruster-specifications
  parameter SI.ElectricalTorqueConstant k(start = 1) = 1 "Transformation coefficient" annotation(Dialog(tab = "Motor Specific"));
  parameter SI.Resistance R(start = 1) = 0.18 "Resistance of propeller motor" annotation(Dialog(tab = "Motor Specific"));
  parameter SI.Inductance L(start = 1) = 0.077 "Inductance of propeller motor" annotation(Dialog(tab = "Motor Specific"));
  // parameter for propeller mount
  parameter SI.Mass k_m = 0.1 "Propeller shape coefficient, for representing fluid mass pushed through in some constant time t by propeller" annotation(Dialog(tab = "Propeller Body Specific"));
  parameter SI.Length k_r = 1 "Propeller shape coefficient, for rotation-to-linear conversion in torque-to-force calculation" annotation(Dialog(tab = "Propeller Body Specific"));
  parameter Real eta = 0.85 "Efficiency coefficient of the propeller, to translate torque to force" annotation(Dialog(tab = "Propeller Body Specific"));
  parameter Real w_shift = 0.001 "Coefficient of angular velocity in power balance to prevent divide-by-zero error" annotation(Dialog(tab = "Propeller Body Specific"));
  parameter Modelica.Mechanics.MultiBody.Types.Axis n = {1, 0, 0} "Axis of rotation = axis of support torque (resolved in frame_a)" annotation(Evaluate = true, Dialog(tab = "Propeller Body Specific"));
  // parameters for propeller body
  parameter SI.Length r_CM_Propeller[3] = {0, 0, 0} "r_CM from frame_a to center of mass of propeller";
  parameter SI.Mass m_Propeller = 0.3603 "Mass of propeller body";
  parameter SI.Inertia j_Propeller = 0.000001 "Inertia of propeller body";
  parameter SI.Density d_Propeller = 1880 "Average density of propeller body";
  parameter SI.Area A_Propeller = 0.2 "Overall cross sectional area effective in drag of propeller body";
  parameter SI.DimensionlessRatio mu_d_Propeller = 1 "Drag coefficient of the propeller body";
  parameter SI.RotationalDampingConstant k_d_Propeller = 1 "Drag coefficient of torque on propeller body";
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end T200Parameters;
