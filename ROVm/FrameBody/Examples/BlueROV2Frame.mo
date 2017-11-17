within ROVm.FrameBody.Examples;

model BlueROV2Frame
  inner RBodyInFluid.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-132.079, -35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-131.832, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  SideFrame.SidePlate leftSide(BP_Pos = {0.15, 0, 0}, TP_Pos = {0.4, 0, 0}) annotation(Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  SideFrame.SidePlate rightSide(TP_Pos = {0.4, 0, 0}, BP_Pos = {0.15, 0, 0}) annotation(Placement(visible = true, transformation(origin = {75, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  TopFrame.TopPiece topPiece annotation(Placement(visible = true, transformation(origin = {-40, 60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  TopFrame.TopPiece topPiece1 annotation(Placement(visible = true, transformation(origin = {-43.139, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Electronics.ElectronicsEnclosure.EEnclosure eEnclosure(r_CM_EEnclosure = {1, 0, 0}, lf_pos = {0.9, 0, -0.1}, lb_pos = {0.1, 0, -0.1}, rf_pos = {0.9, 0, 0.1}, rb_pos = {0.1, 0, 0.1}, d_EEnclosure = 900, m_EEnclosure = 2, A_EEnclosure = 0.3, c_d_EEnclosure = 5, R_EEnclosure = 100) annotation(Placement(visible = true, transformation(origin = {-20, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  TopFrame.TopPiece topPiece2 annotation(Placement(visible = true, transformation(origin = {20, 60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  BottomFrame.BottomPlate bottomPlate(r_bP_Long = {0.6, 0, 0}, innerRotationAxis = {0, -1, 0}) annotation(Placement(visible = true, transformation(origin = {10, -86.894}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  TopFrame.TopPiece topPiece3 annotation(Placement(visible = true, transformation(origin = {45, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Electronics.Battery.Battery battery(r_CM_Battery = {0, 0.1, 0}, m_Battery = 3, d_Battery = 5000, A_Battery = 0.25, c_d_Battery = 1, C_Battery = 18) annotation(Placement(visible = true, transformation(origin = {12.761, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(leftSide.frame_frontU, topPiece.frame_SF) annotation(Line(visible = true, origin = {-66.113, 38.238}, points = {{-8.071, -29.238}, {-8.071, 14.619}, {16.143, 14.619}}, color = {95, 95, 95}));
  connect(leftSide.frame_backU, topPiece1.frame_SF) annotation(Line(visible = true, origin = {-67.303, -18.238}, points = {{-7.097, 9.238}, {-7.097, -4.619}, {14.194, -4.619}}, color = {95, 95, 95}));
  connect(topPiece.frame_EC, eEnclosure.frame_lf) annotation(Line(visible = true, origin = {-29.762, 39.952}, points = {{-0.208, 12.905}, {3.462, 12.905}, {3.462, 7.048}, {-3.238, 7.048}, {-3.238, -19.952}, {-0.238, -19.952}}, color = {95, 95, 95}));
  connect(topPiece1.frame_EC, eEnclosure.frame_lb) annotation(Line(visible = true, origin = {-31.331, -10.619}, points = {{-1.777, -12.238}, {1.892, -12.238}, {1.892, -6.381}, {-1.669, -6.381}, {-1.669, 18.619}, {1.331, 18.619}}, color = {95, 95, 95}));
  connect(topPiece2.frame_EC, rightSide.frame_backU) annotation(Line(visible = true, origin = {56.277, 38.238}, points = {{-26.247, 14.619}, {13.123, 14.619}, {13.123, -29.238}}, color = {95, 95, 95}));
  connect(topPiece2.frame_SF, eEnclosure.frame_rf) annotation(Line(visible = true, origin = {-3.492, 36.429}, points = {{13.523, 16.429}, {-3.508, 16.429}, {-3.508, -16.429}, {-6.508, -16.429}}, color = {95, 95, 95}));
  connect(leftSide.frame_frontL, bottomPlate.frame_lf) annotation(Line(visible = true, origin = {-80.483, -25.22}, points = {{-5.74, 32.62}, {-5.74, 38.22}, {-34.517, 38.22}, {-34.517, -54.531}, {80.513, -54.531}}, color = {95, 95, 95}));
  connect(leftSide.frame_backL, bottomPlate.frame_lb) annotation(Line(visible = true, origin = {-78.467, -46.111}, points = {{-7.716, 37.711}, {-7.716, 29.546}, {-31.533, 29.546}, {-31.533, -48.402}, {78.497, -48.402}}, color = {95, 95, 95}));
  connect(rightSide.frame_frontL, bottomPlate.frame_rf) annotation(Line(visible = true, origin = {79.703, -38.58}, points = {{1.52, 31.18}, {1.52, 25.58}, {28.316, 25.58}, {28.316, -41.171}, {-59.673, -41.171}}, color = {95, 95, 95}));
  connect(rightSide.frame_backL, bottomPlate.frame_rb) annotation(Line(visible = true, origin = {83.684, -30.918}, points = {{-2.501, 39.318}, {-2.501, 43.918}, {34.343, 43.918}, {34.343, -63.576}, {-63.684, -63.576}}, color = {95, 95, 95}));
  connect(topPiece3.frame_EC, rightSide.frame_frontU) annotation(Line(visible = true, origin = {64.466, -18.238}, points = {{-9.436, -4.619}, {4.718, -4.619}, {4.718, 9.238}}, color = {95, 95, 95}));
  connect(topPiece3.frame_SF, eEnclosure.frame_rb) annotation(Line(visible = true, origin = {2.758, -7.429}, points = {{32.273, -15.429}, {-9.758, -15.429}, {-9.758, 15.429}, {-12.758, 15.429}}, color = {95, 95, 95}));
  connect(battery.pin_p, eEnclosure.pin_p) annotation(Line(visible = true, origin = {-13.497, -8.157}, points = {{16.258, -31.843}, {16.258, -28.843}, {18.497, -28.843}, {18.497, 11.608}, {-26.503, 11.608}, {-26.503, 33.157}, {-16.503, 33.157}}, color = {10, 90, 224}));
  connect(battery.frame_a, bottomPlate.frame_bat) annotation(Line(visible = true, origin = {4.457, -64.936}, points = {{-1.696, 14.936}, {-4.696, 14.936}, {-4.696, -8.958}, {5.543, -8.958}, {5.543, -11.958}}, color = {95, 95, 95}));
  connect(eEnclosure.pin_n, battery.pin_n) annotation(Line(visible = true, origin = {16.071, -7.5}, points = {{-26.071, 32.5}, {9.69, 32.5}, {9.69, -32.5}, {6.69, -32.5}}, color = {10, 90, 224}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end BlueROV2Frame;
