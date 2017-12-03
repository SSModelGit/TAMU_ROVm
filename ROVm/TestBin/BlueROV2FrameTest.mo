within ROVm.TestBin;

model BlueROV2FrameTest
  inner RBodyInFluid.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-132.079, -35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-131.832, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FrameBody.BottomFrame.BottomPlate bottomPlate(r_bP_Long = {0, 0, 0.7}, c_d_bP_Long = 0, c_d_bP_Short = 0) annotation(Placement(visible = true, transformation(origin = {-2.071, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FrameBody.SideFrame.SidePlate leftSide1(color = {0, 180, 0}, c_d_SP = 0) annotation(Placement(visible = true, transformation(origin = {65, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Electronics.Battery.Battery battery(r_CM_Battery = {0, 0.1, 0}, m_Battery = 1.452, d_Battery = 725.6, A_Battery = 0.25, c_d_Battery = 0, C_Battery = 18) annotation(Placement(visible = true, transformation(origin = {12.761, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FrameBody.TopFrame.TopPiece topPiece3(c_d_TP = 0, c_d_Fairing = 0) annotation(Placement(visible = true, transformation(origin = {45, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FrameBody.TopFrame.TopPiece topPiece2(c_d_TP = 0, c_d_Fairing = 0) annotation(Placement(visible = true, transformation(origin = {20, 60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Electronics.ElectronicsEnclosure.EEnclosure eEnclosure(r_CM_EEnclosure = {0.5, 0, 0}, lf_pos = {0.9, 0, -0.2}, lb_pos = {0.1, 0, -0.2}, rf_pos = {0.9, 0, 0.2}, rb_pos = {0.1, 0, 0.2}, d_EEnclosure = 900, m_EEnclosure = 2, A_EEnclosure = 0.3, c_d_EEnclosure = 0, R_EEnclosure = 100, color = {255, 0, 0}) annotation(Placement(visible = true, transformation(origin = {-20, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FrameBody.TopFrame.TopPiece topPiece1(c_d_TP = 0, c_d_Fairing = 0) annotation(Placement(visible = true, transformation(origin = {-43.139, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FrameBody.TopFrame.TopPiece topPiece(c_d_TP = 0, c_d_Fairing = 0) annotation(Placement(visible = true, transformation(origin = {-40, 60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  FrameBody.SideFrame.SidePlate leftSide(color = {0, 0, 0}, c_d_SP = 0) annotation(Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Electronics.Lumens.Lumen lumen(r_CM_Lumen = {0.02, 0, 0}, m_Lumen = 0.05, d_Lumen = 5000, A_Lumen = 0.0003, c_d_Lumen = 1, R_Lumen = 5, color = {255, 0, 255}) annotation(Placement(visible = true, transformation(origin = {-100, 33.476}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Electronics.Lumens.Lumen lumen1(r_CM_Lumen = {0.02, 0, 0}, m_Lumen = 0.05, d_Lumen = 5000, A_Lumen = 0.0003, c_d_Lumen = 1, R_Lumen = 5, color = {255, 0, 255}) annotation(Placement(visible = true, transformation(origin = {-100, -30}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  Electronics.Lumens.Lumen lumen2(r_CM_Lumen = {0.02, 0, 0}, m_Lumen = 0.05, d_Lumen = 5000, A_Lumen = 0.0003, c_d_Lumen = 1, R_Lumen = 5, color = {255, 0, 255}) annotation(Placement(visible = true, transformation(origin = {92.312, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Electronics.Lumens.Lumen lumen3(r_CM_Lumen = {0.02, 0, 0}, m_Lumen = 0.05, d_Lumen = 5000, A_Lumen = 0.0003, c_d_Lumen = 1, R_Lumen = 5, color = {255, 0, 255}) annotation(Placement(visible = true, transformation(origin = {92.312, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(battery.pin_p, eEnclosure.pin_p) annotation(Line(visible = true, origin = {-13.62, 10.475}, points = {{16.38, -50.475}, {16.38, 17.975}, {-16.38, 17.975}, {-16.38, 14.525}}, color = {10, 90, 224}));
  connect(topPiece.frame_EC, eEnclosure.frame_lf) annotation(Line(visible = true, origin = {-29.762, 39.952}, points = {{-0.208, 12.905}, {3.462, 12.905}, {3.462, 7.048}, {-3.238, 7.048}, {-3.238, -19.952}, {-0.238, -19.952}}, color = {95, 95, 95}));
  connect(topPiece1.frame_EC, eEnclosure.frame_lb) annotation(Line(visible = true, origin = {-31.331, -10.619}, points = {{-1.778, -12.238}, {1.892, -12.238}, {1.892, -6.381}, {-1.669, -6.381}, {-1.669, 18.619}, {1.331, 18.619}}, color = {95, 95, 95}));
  connect(eEnclosure.frame_rf, topPiece2.frame_SF) annotation(Line(visible = true, origin = {3.158, 36.429}, points = {{-13.158, -16.429}, {3.142, -16.429}, {3.142, 16.429}, {6.873, 16.429}}, color = {95, 95, 95}));
  connect(eEnclosure.frame_rb, topPiece3.frame_SF) annotation(Line(visible = true, origin = {21.908, -7.429}, points = {{-31.908, 15.429}, {9.392, 15.429}, {9.392, -15.429}, {13.123, -15.429}}, color = {95, 95, 95}));
  connect(leftSide.frame_backL, bottomPlate.frame_lb) annotation(Line(visible = true, origin = {-61.469, -61.213}, points = {{-24.714, 52.813}, {-24.714, -26.406}, {49.428, -26.406}}, color = {95, 95, 95}));
  connect(leftSide1.frame_backL, bottomPlate.frame_rb) annotation(Line(visible = true, origin = {41.854, -57.867}, points = {{16.963, 59.467}, {16.963, -29.733}, {-33.925, -29.733}}, color = {95, 95, 95}));
  connect(bottomPlate.frame_bat, battery.frame_a) annotation(Line(visible = true, origin = {-0.46, -56.667}, points = {{-1.611, -13.333}, {-1.611, 6.667}, {3.221, 6.667}}, color = {95, 95, 95}));
  connect(leftSide1.frame_frontU, topPiece2.frame_EC) annotation(Line(visible = true, origin = {57.221, 41.571}, points = {{13.595, -22.571}, {13.595, 11.286}, {-27.19, 11.286}}, color = {95, 95, 95}));
  connect(lumen.frame_a, leftSide.lumenU) annotation(Line(visible = true, origin = {-72.36, 20.689}, points = {{-17.64, 12.787}, {7.64, 12.787}, {7.64, -12.787}, {2.36, -12.787}}, color = {95, 95, 95}));
  connect(lumen1.frame_a, leftSide.lumenL) annotation(Line(visible = true, origin = {-72.321, -19.04}, points = {{-17.679, -10.96}, {7.601, -10.96}, {7.601, 10.96}, {2.477, 10.96}}, color = {95, 95, 95}));
  connect(lumen2.frame_a, leftSide1.lumenU) annotation(Line(visible = true, origin = {79.333, 28.217}, points = {{2.979, 11.783}, {-0.271, 11.783}, {-0.271, -1.467}, {0.947, -1.467}, {0.947, -10.316}, {-4.333, -10.316}}, color = {95, 95, 95}));
  connect(lumen3.frame_a, leftSide1.lumenL) annotation(Line(visible = true, origin = {79.359, -8.277}, points = {{2.953, -11.723}, {-0.297, -11.723}, {-0.297, 1.527}, {0.921, 1.527}, {0.921, 10.196}, {-4.203, 10.196}}, color = {95, 95, 95}));
  connect(eEnclosure.pin_n, lumen.pin_p) annotation(Line(visible = true, origin = {-28.5, 34.238}, points = {{18.5, -9.238}, {21.5, -9.238}, {21.5, 9.238}, {-61.5, 9.238}}, color = {10, 90, 224}));
  connect(lumen.pin_n, lumen1.pin_p) annotation(Line(visible = true, origin = {-96.7, 11.386}, points = {{-13.3, 32.09}, {-13.3, 35.34}, {9.95, 35.34}, {9.95, -51.386}, {6.7, -51.386}}, color = {10, 90, 224}));
  connect(lumen1.pin_n, lumen3.pin_p) annotation(Line(visible = true, origin = {4.087, -29.3}, points = {{-114.087, -10.7}, {-114.087, -13.95}, {74.975, -13.95}, {74.975, 19.3}, {78.225, 19.3}}, color = {10, 90, 224}));
  connect(lumen3.pin_n, lumen2.pin_p) annotation(Line(visible = true, origin = {89.012, 15.3}, points = {{13.3, -25.3}, {13.3, -22.05}, {-9.95, -22.05}, {-9.95, 34.7}, {-6.7, 34.7}}, color = {10, 90, 224}));
  connect(lumen2.pin_n, battery.pin_n) annotation(Line(visible = true, origin = {55.781, 15.3}, points = {{46.531, 34.7}, {46.531, 37.95}, {-30.02, 37.95}, {-30.02, -55.3}, {-33.02, -55.3}}, color = {10, 90, 224}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end BlueROV2FrameTest;