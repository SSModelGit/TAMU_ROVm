within ROVm.FrameBody.Examples;

model BlueROV2Frame
  inner UnderwaterRigidBodyLibrary.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-132.079, -35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-131.832, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  SideFrame.SidePlate leftSide annotation(Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  TopFrame.TopPiece topPiece annotation(Placement(visible = true, transformation(origin = {-40, 60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  TopFrame.TopPiece topPiece1 annotation(Placement(visible = true, transformation(origin = {-43.139, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Electronics.ElectronicsEnclosure.EEnclosure eEnclosure(r_CM_EEnclosure = {0.5, 0, 0}, lf_pos = {0.9, 0, -0.2}, lb_pos = {0.1, 0, -0.2}, rf_pos = {0.9, 0, 0.2}, rb_pos = {0.1, 0, 0.2}, d_EEnclosure = 900, m_EEnclosure = 2, A_EEnclosure = 0.3, mu_d_EEnclosure = 5, R_EEnclosure = 100) annotation(Placement(visible = true, transformation(origin = {-20, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  TopFrame.TopPiece topPiece2 annotation(Placement(visible = true, transformation(origin = {20, 60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  TopFrame.TopPiece topPiece3 annotation(Placement(visible = true, transformation(origin = {45, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Electronics.Battery.Battery battery(r_CM_Battery = {0, 0.1, 0}, m_Battery = 3, d_Battery = 5000, A_Battery = 0.25, mu_d_Battery = 1, C_Battery = 18) annotation(Placement(visible = true, transformation(origin = {12.761, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  SideFrame.SidePlate leftSide1 annotation(Placement(visible = true, transformation(origin = {65, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  BottomFrame.BottomPlate bottomPlate(r_bP_Long = {0, 0, 0.7}) annotation(Placement(visible = true, transformation(origin = {-2.071, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  UnderwaterRigidBodyLibrary.Parts.BasicBody basicBody(mu_d = 1, A = 1, r_CM = {0, 0, 0}, m = 5) annotation(Placement(visible = true, transformation(origin = {-126.655, 83.171}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(battery.pin_p, eEnclosure.pin_p) annotation(Line(visible = true, origin = {-13.497, -8.157}, points = {{16.258, -31.843}, {16.258, -28.843}, {18.497, -28.843}, {18.497, 11.608}, {-26.503, 11.608}, {-26.503, 33.157}, {-16.503, 33.157}}, color = {10, 90, 224}));
  connect(eEnclosure.pin_n, battery.pin_n) annotation(Line(visible = true, origin = {16.071, -7.5}, points = {{-26.071, 32.5}, {9.69, 32.5}, {9.69, -32.5}, {6.69, -32.5}}, color = {10, 90, 224}));
  connect(topPiece.frame_EC, eEnclosure.frame_lf) annotation(Line(visible = true, origin = {-29.762, 39.952}, points = {{-0.208, 12.905}, {3.462, 12.905}, {3.462, 7.048}, {-3.238, 7.048}, {-3.238, -19.952}, {-0.238, -19.952}}, color = {95, 95, 95}));
  connect(topPiece1.frame_EC, eEnclosure.frame_lb) annotation(Line(visible = true, origin = {-31.331, -10.619}, points = {{-1.778, -12.238}, {1.892, -12.238}, {1.892, -6.381}, {-1.669, -6.381}, {-1.669, 18.619}, {1.331, 18.619}}, color = {95, 95, 95}));
  connect(bottomPlate.frame_bat, battery.frame_a) annotation(Line(visible = true, origin = {-0.46, -56.667}, points = {{-1.611, -13.333}, {-1.611, 6.667}, {3.221, 6.667}}, color = {95, 95, 95}));
  connect(eEnclosure.frame_rf, topPiece2.frame_SF) annotation(Line(visible = true, origin = {3.158, 36.429}, points = {{-13.158, -16.429}, {3.142, -16.429}, {3.142, 16.429}, {6.873, 16.429}}, color = {95, 95, 95}));
  connect(eEnclosure.frame_rb, topPiece3.frame_SF) annotation(Line(visible = true, origin = {21.908, -7.429}, points = {{-31.908, 15.429}, {9.392, 15.429}, {9.392, -15.429}, {13.123, -15.429}}, color = {95, 95, 95}));
  connect(leftSide1.frame_frontU, topPiece.frame_SF) annotation(Line(visible = true, origin = {-3.148, 34.143}, points = {{73.963, -15.143}, {73.963, -11.143}, {-50.552, -11.143}, {-50.552, 18.714}, {-46.822, 18.714}}, color = {95, 95, 95}));
  connect(leftSide1.frame_backU, topPiece1.frame_SF) annotation(Line(visible = true, origin = {-5.117, -10.143}, points = {{75.717, 11.143}, {75.717, 7.143}, {-51.722, 7.143}, {-51.722, -12.714}, {-47.991, -12.714}}, color = {95, 95, 95}));
  connect(leftSide.frame_backU, topPiece3.frame_EC) annotation(Line(visible = true, origin = {4.726, -16.143}, points = {{-79.126, 7.143}, {-79.126, 3.143}, {53.974, 3.143}, {53.974, -6.714}, {50.304, -6.714}}, color = {95, 95, 95}));
  connect(bottomPlate.frame_lb, leftSide1.frame_backL) annotation(Line(visible = true, origin = {14.996, -35.928}, points = {{-27.037, -51.691}, {-30.302, -51.691}, {-30.302, 32.928}, {43.82, 32.928}, {43.82, 37.528}}, color = {95, 95, 95}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end BlueROV2Frame;
