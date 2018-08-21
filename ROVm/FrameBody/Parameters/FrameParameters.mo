within ROVm.FrameBody.Parameters;

record FrameParameters "Contains common parameters for frames"
  import SI = Modelica.SIunits;
  parameter SI.Density d_HDPE = 970;
  // Top plate params
  parameter SI.Length r_TP[3] = {0, 0, 0.15} "Total length of top piece" annotation(Dialog(tab = "Top Plate Specific"));
  parameter SI.Length r_CM_TP[3] = r_TP / 2 annotation(Dialog(tab = "Top Plate Specific"));
  parameter SI.Mass m_TP = 0.1 annotation(Dialog(tab = "Top Plate Specific"));
  parameter SI.Area A_TP = 0.0009 annotation(Dialog(tab = "Top Plate Specific"));
  parameter SI.DimensionlessRatio mu_d_TP = 0.15 annotation(Dialog(tab = "Top Plate Specific"));
  parameter SI.RotationalDampingConstant k_d_TP = 10000 annotation(Dialog(tab = "Top Plate Specific"));
  parameter SI.Length r_CM_Fairing[3] = {0, 0.05, 0} annotation(Dialog(tab = "Top Plate Specific"));
  parameter SI.Length r_Fairing[3] = r_TP annotation(Dialog(tab = "Top Plate Specific"));
  parameter SI.Density d_Fairing = 288 annotation(Dialog(tab = "Top Plate Specific"));
  parameter SI.Mass m_Fairing(displayUnit = "g") = 0.148 annotation(Dialog(tab = "Top Plate Specific"));
  parameter SI.Area A_Fairing = 0.002 annotation(Dialog(tab = "Top Plate Specific"));
  parameter SI.DimensionlessRatio mu_d_Fairing = 0.1 annotation(Dialog(tab = "Top Plate Specific"));
  parameter SI.RotationalDampingConstant k_d_Fairing = 10000 annotation(Dialog(tab = "Top Plate Specific"));
  parameter SI.Length thickness[3] = {0, 0.0127, 0} annotation(Dialog(tab = "Top Plate Specific"));
  // Side plate params
  parameter Boolean secondLumen = true "true = Use of second lumen, position = -lumenUPos" annotation(Dialog(tab = "Side Plate Specific"));
  parameter SI.Length r_SP[3] = {1, 0, 0} "Total length of Side Plate, from frame_a to frame_b" annotation(Dialog(tab = "Side Plate Specific"));
  parameter SI.Length r_CM_SP[3] = r_SP / 2 "Length from frame_a to CM of Side Plate" annotation(Dialog(tab = "Side Plate Specific"));
  parameter SI.Mass m_SP = 0.5 annotation(Dialog(tab = "Side Plate Specific"));
  parameter SI.Area A_SP = 0.5 annotation(Dialog(tab = "Side Plate Specific"));
  parameter SI.DimensionlessRatio mu_d_SP = 1 annotation(Dialog(tab = "Side Plate Specific"));
  parameter SI.RotationalDampingConstant k_d_SP = 10000 annotation(Dialog(tab = "Side Plate Specific"));
  parameter SI.Length TP_Pos[3] = {0.4, 0, 0} "Position of front side top piece connection point, from center of mass of side plate" annotation(Dialog(tab = "Side Plate Specific"));
  parameter SI.Length BP_Pos[3] = {0.15, 0, 0} "Position of back side bottom plate connection point, from center of mass of side plate" annotation(Dialog(tab = "Side Plate Specific"));
  parameter SI.Length propVPos[3] = {0.40, 0.2, 0} + r_CM_TP "Position of vertical propeller from frame_a" annotation(Dialog(tab = "Side Plate Specific"));
  parameter SI.Length lumenUPos[3] = {0, 0.3, 0} "Position of upper (or only) lumen from frame_a" annotation(Dialog(tab = "Side Plate Specific"));
  // Bottom plate params
  parameter SI.Length r_bP_Long[3] = {0, 0, 0.7} annotation(Dialog(tab = "Bottom Plate Specific"));
  parameter SI.Length r_CM_bP_Long[3] = r_bP_Long / 2 annotation(Dialog(tab = "Bottom Plate Specific"));
  parameter SI.Mass m_bP_Long = 0.1 annotation(Dialog(tab = "Bottom Plate Specific"));
  parameter SI.Area A_bP_Long = 0.0009 annotation(Dialog(tab = "Bottom Plate Specific"));
  parameter SI.DimensionlessRatio mu_d_bP_Long = 0.5 annotation(Dialog(tab = "Bottom Plate Specific"));
  parameter SI.RotationalDampingConstant k_d_bP_Long = 10000 annotation(Dialog(tab = "Bottom Plate Specific"));
  parameter SI.DimensionlessRatio mu_d_bP_Short = 0.2 annotation(Dialog(tab = "Bottom Plate Specific"));
  parameter SI.RotationalDampingConstant k_d_bP_Short = 10000 annotation(Dialog(tab = "Bottom Plate Specific"));
  parameter Real innerScaleFactor = 2 annotation(Dialog(tab = "Bottom Plate Specific"));
  parameter SI.Length r_bP_Short[3] = {0.3, 0, 0} annotation(Dialog(tab = "Bottom Plate Specific"));
  parameter Modelica.Mechanics.MultiBody.Types.Axis innerRotationAxis "Think wisely, grasshopper." annotation(Dialog(tab = "Bottom Plate Specific"));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end FrameParameters;
