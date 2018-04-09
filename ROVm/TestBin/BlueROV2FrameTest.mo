within ROVm.TestBin;

model BlueROV2FrameTest
  inner RBodyInFluid.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-117.079, -140}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-116.832, -105}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FrameBody.BottomFrame.BottomPlate bottomPlate(r_bP_Long = frameParams.r_bP_Long, c_d_bP_Long = frameParams.c_d_bP_Long, c_d_bP_Short = frameParams.c_d_bP_Short, d_plate = frameParams.d_HDPE, r_CM_bP_Long = frameParams.r_CM_bP_Long, m_bP_Long = frameParams.m_bP_Long, A_bP_Long = frameParams.A_bP_Long, innerScaleFactor = frameParams.innerScaleFactor, r_bP_Short = frameParams.r_bP_Short) annotation(Placement(visible = true, transformation(origin = {12.929, -185}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FrameBody.SideFrame.SidePlate leftSide1(color = {0, 0, 0}, c_d_SP = frameParams.c_d_SP, secondLumen = frameParams.secondLumen, r_SP = frameParams.r_SP, r_CM_SP = frameParams.r_CM_SP, d_plate = frameParams.d_HDPE, m_SP = frameParams.m_SP, A_SP = frameParams.A_SP, TP_Pos = frameParams.TP_Pos, BP_Pos = frameParams.BP_Pos, propVPos = frameParams.propVPos, lumenUPos = frameParams.lumenUPos) annotation(Placement(visible = true, transformation(origin = {80, -95}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Electronics.Battery.Battery battery(r_CM_Battery = {0, 0.1, 0}, m_Battery = 1.452, d_Battery = 725.6, A_Battery = 0.25, c_d_Battery = 0, C_Battery = 18, V_Battery = 14.8) annotation(Placement(visible = true, transformation(origin = {27.761, -155}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FrameBody.TopFrame.TopPiece topPiece3(c_d_TP = frameParams.c_d_TP, c_d_Fairing = frameParams.c_d_Fairing, d_plate = frameParams.d_HDPE, r_TP = frameParams.r_TP, TP_Width = frameParams.TP_Width, r_CM_TP = frameParams.r_CM_TP, r_CM_Fairing = frameParams.r_CM_Fairing, m_TP = frameParams.m_TP, A_TP = frameParams.A_TP, thickness = frameParams.thickness, r_Fairing = frameParams.r_Fairing, d_Fairing = frameParams.d_Fairing, m_Fairing = frameParams.m_Fairing, A_Fairing = frameParams.A_Fairing) annotation(Placement(visible = true, transformation(origin = {60, -135}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FrameBody.TopFrame.TopPiece topPiece1(c_d_TP = frameParams.c_d_TP, c_d_Fairing = frameParams.c_d_Fairing, d_plate = frameParams.d_HDPE, r_TP = frameParams.r_TP, TP_Width = frameParams.TP_Width, r_CM_TP = frameParams.r_CM_TP, r_CM_Fairing = frameParams.r_CM_Fairing, m_TP = frameParams.m_TP, A_TP = frameParams.A_TP, thickness = frameParams.thickness, r_Fairing = frameParams.r_Fairing, d_Fairing = frameParams.d_Fairing, m_Fairing = frameParams.m_Fairing, A_Fairing = frameParams.A_Fairing) annotation(Placement(visible = true, transformation(origin = {35, -45}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Electronics.ElectronicsEnclosure.EEnclosure eEnclosure(r_CM_EEnclosure = eeParams.r_CM_EEnclosure, lf_pos = eeParams.lf_pos, lb_pos = eeParams.lb_pos, rf_pos = eeParams.rf_pos, rb_pos = eeParams.rb_pos, d_EEnclosure = eeParams.d_EEnclosure, m_EEnclosure = eeParams.m_EEnclosure, A_EEnclosure = eeParams.A_EEnclosure, c_d_EEnclosure = eeParams.c_d_EEnclosure, R_EEnclosure = eeParams.R_EEnclosure, color = {255, 65, 65}) annotation(Placement(visible = true, transformation(origin = {-5, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FrameBody.TopFrame.TopPiece topPiece4(c_d_TP = frameParams.c_d_TP, c_d_Fairing = frameParams.c_d_Fairing, d_plate = frameParams.d_HDPE, r_TP = frameParams.r_TP, TP_Width = frameParams.TP_Width, r_CM_TP = frameParams.r_CM_TP, r_CM_Fairing = frameParams.r_CM_Fairing, m_TP = frameParams.m_TP, A_TP = frameParams.A_TP, thickness = frameParams.thickness, r_Fairing = frameParams.r_Fairing, d_Fairing = frameParams.d_Fairing, m_Fairing = frameParams.m_Fairing, A_Fairing = frameParams.A_Fairing) annotation(Placement(visible = true, transformation(origin = {-28.139, -135}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FrameBody.TopFrame.TopPiece topPiece2(c_d_TP = frameParams.c_d_TP, c_d_Fairing = frameParams.c_d_Fairing, d_plate = frameParams.d_HDPE, r_TP = frameParams.r_TP, TP_Width = frameParams.TP_Width, r_CM_TP = frameParams.r_CM_TP, r_CM_Fairing = frameParams.r_CM_Fairing, m_TP = frameParams.m_TP, A_TP = frameParams.A_TP, thickness = frameParams.thickness, r_Fairing = frameParams.r_Fairing, d_Fairing = frameParams.d_Fairing, m_Fairing = frameParams.m_Fairing, A_Fairing = frameParams.A_Fairing) annotation(Placement(visible = true, transformation(origin = {-27.791, -45}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  FrameBody.SideFrame.SidePlate leftSide(color = {0, 0, 0}, c_d_SP = frameParams.c_d_SP, r_SP = frameParams.r_SP, r_CM_SP = frameParams.r_CM_SP, d_plate = frameParams.d_HDPE, m_SP = frameParams.m_SP, A_SP = frameParams.A_SP, TP_Pos = frameParams.TP_Pos, BP_Pos = frameParams.BP_Pos, propVPos = frameParams.propVPos, lumenUPos = frameParams.lumenUPos, secondLumen = frameParams.secondLumen) annotation(Placement(visible = true, transformation(origin = {-65, -105}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Electronics.Lumens.Lumen lumen(r_CM_Lumen = {0.02, 0, 0}, m_Lumen = 0.05, d_Lumen = 1000, A_Lumen = 0.0003, c_d_Lumen = 0, R_Lumen = 5, color = {255, 0, 255}) annotation(Placement(visible = true, transformation(origin = {-85, -71.524}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Electronics.Lumens.Lumen lumen1(r_CM_Lumen = {0.02, 0, 0}, m_Lumen = 0.05, d_Lumen = 1000, A_Lumen = 0.0003, c_d_Lumen = 0, R_Lumen = 5, color = {255, 0, 255}) annotation(Placement(visible = true, transformation(origin = {-85, -135}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  Electronics.Lumens.Lumen lumen2(r_CM_Lumen = {0.02, 0, 0}, m_Lumen = 0.05, d_Lumen = 1000, A_Lumen = 0.0003, c_d_Lumen = 0, R_Lumen = 5, color = {255, 0, 255}) annotation(Placement(visible = true, transformation(origin = {107.312, -65}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Electronics.Lumens.Lumen lumen3(r_CM_Lumen = {0.02, 0, 0}, m_Lumen = 0.05, d_Lumen = 1000, A_Lumen = 0.0003, c_d_Lumen = 0, R_Lumen = 5, color = {255, 0, 255}) annotation(Placement(visible = true, transformation(origin = {107.657, -128.451}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Propeller.Examples.BasicProp basicProp2(m_Propeller = propParams.m_Propeller, A_Propeller = propParams.A_Propeller, n = Modelica.Math.Vectors.normalize({1, 0, 1}), r = propParams.r, direction = -1, r_CM_Propeller = propParams.r_CM_Propeller, j_Propeller = propParams.j_Propeller, d_Propeller = propParams.d_Propeller, c_d_Propeller = propParams.c_d_Propeller, k = propParams.k, R = propParams.R, L = propParams.L, eta = propParams.eta) annotation(Placement(visible = true, transformation(origin = {-40, 125}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder2(T = 1, k = 1) annotation(Placement(visible = true, transformation(origin = {-55, 150}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Propeller.Examples.BasicProp basicProp1(m_Propeller = propParams.m_Propeller, A_Propeller = propParams.A_Propeller, n = Modelica.Math.Vectors.normalize({1, 0, -1}), r = propParams.r, r_CM_Propeller = propParams.r_CM_Propeller, j_Propeller = propParams.j_Propeller, d_Propeller = propParams.d_Propeller, c_d_Propeller = propParams.c_d_Propeller, k = propParams.k, R = propParams.R, L = propParams.L, eta = propParams.eta) annotation(Placement(visible = true, transformation(origin = {41.987, 125}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder1(T = 1, k = 1) annotation(Placement(visible = true, transformation(origin = {26.658, 150}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Propeller.Examples.BasicProp basicProp4(m_Propeller = propParams.m_Propeller, A_Propeller = propParams.A_Propeller, n = Modelica.Math.Vectors.normalize({1, 0, -1}), r = propParams.r, direction = -1, r_CM_Propeller = propParams.r_CM_Propeller, j_Propeller = propParams.j_Propeller, d_Propeller = propParams.d_Propeller, c_d_Propeller = propParams.c_d_Propeller, k = propParams.k, R = propParams.R, L = propParams.L, eta = propParams.eta) annotation(Placement(visible = true, transformation(origin = {-40, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder5(T = 1, k = 1) annotation(Placement(visible = true, transformation(origin = {-55, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Propeller.Examples.BasicProp basicProp3(m_Propeller = propParams.m_Propeller, A_Propeller = propParams.A_Propeller, n = Modelica.Math.Vectors.normalize({1, 0, 1}), r = propParams.r, r_CM_Propeller = propParams.r_CM_Propeller, j_Propeller = propParams.j_Propeller, d_Propeller = propParams.d_Propeller, c_d_Propeller = propParams.c_d_Propeller, k = propParams.k, R = propParams.R, L = propParams.L, eta = propParams.eta) annotation(Placement(visible = true, transformation(origin = {58.932, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder6(T = 1, k = 1) annotation(Placement(visible = true, transformation(origin = {43.603, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Propeller.Examples.BasicProp basicProp5(m_Propeller = propParams.m_Propeller, A_Propeller = propParams.A_Propeller, n = Modelica.Math.Vectors.normalize({0, 1, 0}), r = propParams.r, direction = -1, r_CM_Propeller = propParams.r_CM_Propeller, j_Propeller = propParams.j_Propeller, d_Propeller = propParams.d_Propeller, c_d_Propeller = propParams.c_d_Propeller, k = propParams.k, R = propParams.R, L = propParams.L, eta = propParams.eta) annotation(Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder4(T = 1, k = 1) annotation(Placement(visible = true, transformation(origin = {-55, 75}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Propeller.Examples.BasicProp basicProp6(m_Propeller = propParams.m_Propeller, A_Propeller = propParams.A_Propeller, n = Modelica.Math.Vectors.normalize({0, 1, 0}), r = propParams.r, r_CM_Propeller = propParams.r_CM_Propeller, j_Propeller = propParams.j_Propeller, d_Propeller = propParams.d_Propeller, c_d_Propeller = propParams.c_d_Propeller, k = propParams.k, R = propParams.R, L = propParams.L, eta = propParams.eta) annotation(Placement(visible = true, transformation(origin = {58.932, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder3(T = 1, k = 1) annotation(Placement(visible = true, transformation(origin = {43.603, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant power(k = 0) annotation(Placement(visible = true, transformation(origin = {-110, 150}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter FrameBody.Parameters.FrameParameters frameParams "Record containing all parameters specific to the frame" annotation(Dialog(group = "Frame Parameters"));
  parameter Propeller.Parameters.T200Parameters propParams "Record containing parameters specific for the propellers" annotation(Dialog(group = "Propeller Parameters"));
  parameter Electronics.Parameters.ElectronicsEnclosureParameters eeParams "Record containing parameters specific for the Electronics Enclosure" annotation(Dialog(group = "Electronics Enclosure Parameters"));
equation
  connect(battery.pin_p, eEnclosure.pin_p) annotation(Line(visible = true, origin = {1.38, -94.525}, points = {{16.381, -50.475}, {16.38, 17.975}, {-16.38, 17.975}, {-16.38, 14.525}}, color = {10, 90, 224}));
  connect(topPiece2.frame_EC, eEnclosure.frame_lf) annotation(Line(visible = true, origin = {-16.157, -65.048}, points = {{-1.604, 12.905}, {2.066, 12.905}, {2.066, 7.048}, {-1.843, 7.048}, {-1.843, -19.952}, {1.157, -19.952}}, color = {95, 95, 95}));
  connect(topPiece4.frame_EC, eEnclosure.frame_lb) annotation(Line(visible = true, origin = {-16.331, -115.619}, points = {{-1.778, -12.238}, {1.892, -12.238}, {1.892, -6.381}, {-1.669, -6.381}, {-1.669, 18.619}, {1.331, 18.619}}, color = {95, 95, 95}));
  connect(eEnclosure.frame_rf, topPiece1.frame_SF) annotation(Line(visible = true, origin = {18.158, -68.571}, points = {{-13.158, -16.429}, {3.142, -16.429}, {3.142, 16.429}, {6.873, 16.429}}, color = {95, 95, 95}));
  connect(eEnclosure.frame_rb, topPiece3.frame_SF) annotation(Line(visible = true, origin = {36.908, -112.429}, points = {{-31.908, 15.429}, {9.392, 15.429}, {9.392, -15.429}, {13.123, -15.429}}, color = {95, 95, 95}));
  connect(leftSide.frame_backL, bottomPlate.frame_lb) annotation(Line(visible = true, origin = {-46.469, -166.213}, points = {{-24.714, 52.813}, {-24.714, -26.406}, {49.428, -26.406}}, color = {95, 95, 95}));
  connect(leftSide1.frame_backL, bottomPlate.frame_rb) annotation(Line(visible = true, origin = {56.854, -162.867}, points = {{16.963, 59.467}, {16.963, -29.733}, {-33.925, -29.733}}, color = {95, 95, 95}));
  connect(bottomPlate.frame_bat, battery.frame_a) annotation(Line(visible = true, origin = {14.54, -161.667}, points = {{-1.611, -13.333}, {-1.611, 6.667}, {3.221, 6.667}}, color = {95, 95, 95}));
  connect(leftSide1.frame_frontU, topPiece1.frame_EC) annotation(Line(visible = true, origin = {72.221, -63.429}, points = {{13.595, -22.571}, {13.595, 11.286}, {-27.19, 11.286}}, color = {95, 95, 95}));
  connect(lumen.frame_a, leftSide.lumenU) annotation(Line(visible = true, origin = {-57.36, -84.311}, points = {{-17.64, 12.787}, {7.64, 12.787}, {7.64, -12.787}, {2.36, -12.787}}, color = {95, 95, 95}));
  connect(lumen1.frame_a, leftSide.lumenL) annotation(Line(visible = true, origin = {-57.321, -124.04}, points = {{-17.679, -10.96}, {7.601, -10.96}, {7.601, 10.96}, {2.477, 10.959}}, color = {95, 95, 95}));
  connect(lumen2.frame_a, leftSide1.lumenU) annotation(Line(visible = true, origin = {94.333, -76.783}, points = {{2.979, 11.783}, {-0.271, 11.783}, {-0.271, -1.467}, {0.947, -1.467}, {0.947, -10.316}, {-4.333, -10.316}}, color = {95, 95, 95}));
  connect(lumen3.frame_a, leftSide1.lumenL) annotation(Line(visible = true, origin = {94.531, -115.578}, points = {{3.126, -12.873}, {-0.124, -12.873}, {-0.124, 0.377}, {0.749, 0.377}, {0.749, 12.497}, {-4.375, 12.497}}, color = {95, 95, 95}));
  connect(lumen.pin_n, lumen1.pin_p) annotation(Line(visible = true, origin = {-81.7, -93.614}, points = {{-13.3, 32.09}, {-13.3, 35.34}, {9.95, 35.34}, {9.95, -51.386}, {6.7, -51.386}}, color = {10, 90, 224}));
  connect(lumen1.pin_n, lumen3.pin_p) annotation(Line(visible = true, origin = {-23.031, -131.726}, points = {{-71.969, -13.274}, {-71.969, -16.524}, {-48.719, -16.524}, {-48.719, 16.524}, {120.688, 16.524}, {120.688, 13.275}}, color = {10, 90, 224}));
  connect(lumen3.pin_n, lumen2.pin_p) annotation(Line(visible = true, origin = {107.485, -96.217}, points = {{10.172, -22.234}, {13.423, -22.234}, {13.423, -18.984}, {-13.423, -18.984}, {-13.423, 41.217}, {-10.173, 41.217}}, color = {10, 90, 224}));
  connect(eEnclosure.pin_n, lumen.pin_p) annotation(Line(visible = true, origin = {-13.5, -70.762}, points = {{18.5, -9.238}, {21.5, -9.238}, {21.5, 9.238}, {-61.5, 9.238}}, color = {10, 90, 224}));
  connect(lumen2.pin_n, battery.pin_n) annotation(Line(visible = true, origin = {70.781, -89.7}, points = {{46.531, 34.7}, {46.531, 37.95}, {-30.02, 37.95}, {-30.02, -55.3}, {-33.02, -55.3}}, color = {10, 90, 224}));
  connect(firstOrder2.y, basicProp2.u) annotation(Line(visible = true, origin = {-41.333, 144.667}, points = {{-2.667, 5.333}, {1.333, 5.333}, {1.333, -10.667}}, color = {1, 37, 163}));
  connect(firstOrder1.y, basicProp1.u) annotation(Line(visible = true, origin = {40.544, 144.667}, points = {{-2.886, 5.333}, {1.443, 5.333}, {1.443, -10.667}}, color = {1, 37, 163}));
  connect(basicProp1.frame_b, topPiece1.frame_propA) annotation(Line(visible = true, origin = {38.493, 3.958}, points = {{3.494, 111.042}, {3.494, -35.958}, {-3.493, -35.958}, {-3.493, -39.126}}, color = {95, 95, 95}));
  connect(basicProp2.frame_b, topPiece2.frame_propA) annotation(Line(visible = true, origin = {-33.896, 3.958}, points = {{-6.104, 111.042}, {-6.104, -35.958}, {6.104, -35.958}, {6.104, -39.126}}, color = {95, 95, 95}));
  connect(basicProp3.frame_b, topPiece3.frame_propA) annotation(Line(visible = true, origin = {55.077, -49.547}, points = {{3.855, 99.547}, {3.855, 96.322}, {-8.777, 96.322}, {-8.777, -98.453}, {4.923, -98.453}, {4.923, -95.285}}, color = {95, 95, 95}));
  connect(basicProp4.frame_b, topPiece4.frame_propA) annotation(Line(visible = true, origin = {-36.659, -49.547}, points = {{-3.341, 99.547}, {-3.341, 96.322}, {-5.18, 96.322}, {-5.18, -98.453}, {8.52, -98.453}, {8.52, -95.285}}, color = {95, 95, 95}));
  connect(basicProp5.frame_b, leftSide.frame_propV) annotation(Line(visible = true, origin = {-45, -73.333}, points = {{5, 63.333}, {5, -31.667}, {-10, -31.667}}, color = {95, 95, 95}));
  connect(basicProp6.frame_b, leftSide1.frame_propV) annotation(Line(visible = true, origin = {79.685, -45.29}, points = {{-20.753, 35.29}, {-20.753, 32.065}, {15.595, 32.065}, {15.595, -49.71}, {10.315, -49.71}}, color = {95, 95, 95}));
  connect(firstOrder4.y, basicProp4.u) annotation(Line(visible = true, origin = {-42.667, 71}, points = {{-1.333, 4}, {-1.333, -2}, {2.667, -2}}, color = {1, 37, 163}));
  connect(firstOrder5.y, basicProp5.u) annotation(Line(visible = true, origin = {-42.667, 12.667}, points = {{-1.333, 7.333}, {-1.333, -3.667}, {2.667, -3.667}}, color = {1, 37, 163}));
  connect(firstOrder6.y, basicProp6.u) annotation(Line(visible = true, origin = {56.046, 12.667}, points = {{-1.443, 7.333}, {-1.443, -3.667}, {2.886, -3.667}}, color = {1, 37, 163}));
  connect(firstOrder3.y, basicProp3.u) annotation(Line(visible = true, origin = {56.046, 72.667}, points = {{-1.443, 7.333}, {-1.443, -3.667}, {2.886, -3.667}}, color = {1, 37, 163}));
  connect(power.y, firstOrder2.u) annotation(Line(visible = true, origin = {-83, 150}, points = {{-16, 0}, {16, 0}}, color = {1, 37, 163}));
  connect(power.y, firstOrder1.u) annotation(Line(visible = true, origin = {-42.171, 150}, points = {{-56.829, 0}, {56.829, 0}}, color = {1, 37, 163}));
  connect(power.y, firstOrder3.u) annotation(Line(visible = true, origin = {-2.548, 115}, points = {{-96.452, 35}, {31.151, 35}, {31.151, -35}, {34.151, -35}}, color = {1, 37, 163}));
  connect(power.y, firstOrder4.u) annotation(Line(visible = true, origin = {-76.5, 112.5}, points = {{-22.5, 37.5}, {6.5, 37.5}, {6.5, -37.5}, {9.5, -37.5}}, color = {1, 37, 163}));
  connect(power.y, firstOrder5.u) annotation(Line(visible = true, origin = {-76.5, 85}, points = {{-22.5, 65}, {6.5, 65}, {6.5, -65}, {9.5, -65}}, color = {1, 37, 163}));
  connect(power.y, firstOrder6.u) annotation(Line(visible = true, origin = {-2.548, 85}, points = {{-96.452, 65}, {31.151, 65}, {31.151, -65}, {34.151, -65}}, color = {1, 37, 163}));
  annotation(Diagram(coordinateSystem(extent = {{-220, -220}, {220, 220}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end BlueROV2FrameTest;
