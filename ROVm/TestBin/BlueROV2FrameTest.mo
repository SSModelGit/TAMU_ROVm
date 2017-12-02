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
equation
  connect(battery.pin_p, eEnclosure.pin_p) annotation(Line(visible = true, origin = {-13.62, 10.475}, points = {{16.38, -50.475}, {16.38, 17.975}, {-16.38, 17.975}, {-16.38, 14.525}}, color = {10, 90, 224}));
  connect(battery.pin_n, eEnclosure.pin_n) annotation(Line(visible = true, origin = {16.071, -7.5}, points = {{6.69, -32.5}, {9.69, -32.5}, {9.69, 32.5}, {-26.071, 32.5}}, color = {10, 90, 224}));
  connect(topPiece.frame_EC, eEnclosure.frame_lf) annotation(Line(visible = true, origin = {-29.762, 39.952}, points = {{-0.208, 12.905}, {3.462, 12.905}, {3.462, 7.048}, {-3.238, 7.048}, {-3.238, -19.952}, {-0.238, -19.952}}, color = {95, 95, 95}));
  connect(topPiece1.frame_EC, eEnclosure.frame_lb) annotation(Line(visible = true, origin = {-31.331, -10.619}, points = {{-1.778, -12.238}, {1.892, -12.238}, {1.892, -6.381}, {-1.669, -6.381}, {-1.669, 18.619}, {1.331, 18.619}}, color = {95, 95, 95}));
  connect(eEnclosure.frame_rf, topPiece2.frame_SF) annotation(Line(visible = true, origin = {3.158, 36.429}, points = {{-13.158, -16.429}, {3.142, -16.429}, {3.142, 16.429}, {6.873, 16.429}}, color = {95, 95, 95}));
  connect(eEnclosure.frame_rb, topPiece3.frame_SF) annotation(Line(visible = true, origin = {21.908, -7.429}, points = {{-31.908, 15.429}, {9.392, 15.429}, {9.392, -15.429}, {13.123, -15.429}}, color = {95, 95, 95}));
  connect(leftSide.frame_backL, bottomPlate.frame_lb) annotation(Line(visible = true, origin = {-61.469, -61.213}, points = {{-24.714, 52.813}, {-24.714, -26.406}, {49.428, -26.406}}, color = {95, 95, 95}));
  connect(leftSide1.frame_backL, bottomPlate.frame_rb) annotation(Line(visible = true, origin = {41.854, -57.867}, points = {{16.963, 59.467}, {16.963, -29.733}, {-33.925, -29.733}}, color = {95, 95, 95}));
  connect(bottomPlate.frame_bat, battery.frame_a) annotation(Line(visible = true, origin = {-0.46, -56.667}, points = {{-1.611, -13.333}, {-1.611, 6.667}, {3.221, 6.667}}, color = {95, 95, 95}));
  connect(leftSide1.frame_frontU, topPiece2.frame_EC) annotation(Line(visible = true, origin = {57.221, 41.571}, points = {{13.595, -22.571}, {13.595, 11.286}, {-27.19, 11.286}}, color = {95, 95, 95}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end BlueROV2FrameTest;
