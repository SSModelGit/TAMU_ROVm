within ROVm.Examples;

model BasicBlueROV2
  inner RBodyInFluid.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-132.079, -35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-131.832, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.FrameBody.SideFrame.SidePlate leftSide(BP_Pos = {0.15, 0, 0}, TP_Pos = {0.4, 0, 0}) annotation(Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  ROVm.FrameBody.SideFrame.SidePlate rightSide(TP_Pos = {0.4, 0, 0}, BP_Pos = {0.15, 0, 0}) annotation(Placement(visible = true, transformation(origin = {75, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  ROVm.FrameBody.TopFrame.TopPiece topPiece annotation(Placement(visible = true, transformation(origin = {-40, 60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  ROVm.Propeller.Examples.BasicProp basicProp annotation(Placement(visible = true, transformation(origin = {-40, 88.323}, extent = {{-10, -10}, {10, 10}}, rotation = -360)));
  ROVm.Propeller.Examples.BasicProp basicProp1 annotation(Placement(visible = true, transformation(origin = {-60, 21.742}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.Electronics.Lumens.Lumen lumen(r_CM_Lumen = {0.02, 0, 0}, m_Lumen = 0.05, d_Lumen = 5000, A_Lumen = 0.0003, c_d_Lumen = 1, R_Lumen = 5) annotation(Placement(visible = true, transformation(origin = {-95, 33.476}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  ROVm.Electronics.Lumens.Lumen lumen1(r_CM_Lumen = {0.02, 0, 0}, m_Lumen = 0.05, d_Lumen = 5000, A_Lumen = 0.0003, c_d_Lumen = 1, R_Lumen = 5) annotation(Placement(visible = true, transformation(origin = {-95, -30}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  ROVm.FrameBody.TopFrame.TopPiece topPiece1 annotation(Placement(visible = true, transformation(origin = {-43.139, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.Propeller.Examples.BasicProp basicProp2 annotation(Placement(visible = true, transformation(origin = {-43.139, -60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  ROVm.Electronics.ElectronicsEnclosure.EEnclosure eEnclosure(r_CM_EEnclosure = {1, 0, 0}, lf_pos = {0.9, 0, -0.1}, lb_pos = {0.1, 0, -0.1}, rf_pos = {0.9, 0, 0.1}, rb_pos = {0.1, 0, 0.1}, d_EEnclosure = 900, m_EEnclosure = 2, A_EEnclosure = 0.3, c_d_EEnclosure = 5, R_EEnclosure = 100) annotation(Placement(visible = true, transformation(origin = {-20, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.FrameBody.TopFrame.TopPiece topPiece2 annotation(Placement(visible = true, transformation(origin = {20, 60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  ROVm.Propeller.Examples.BasicProp basicProp3 annotation(Placement(visible = true, transformation(origin = {20, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.Electronics.Lumens.Lumen lumen2(r_CM_Lumen = {0.02, 0, 0}, m_Lumen = 0.05, d_Lumen = 5000, A_Lumen = 0.0003, c_d_Lumen = 1, R_Lumen = 5) annotation(Placement(visible = true, transformation(origin = {87.312, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.Electronics.Lumens.Lumen lumen3(r_CM_Lumen = {0.02, 0, 0}, m_Lumen = 0.05, d_Lumen = 5000, A_Lumen = 0.0003, c_d_Lumen = 1, R_Lumen = 5) annotation(Placement(visible = true, transformation(origin = {87.312, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.Propeller.Examples.BasicProp basicProp4 annotation(Placement(visible = true, transformation(origin = {46.59, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.FrameBody.BottomFrame.BottomPlate bottomPlate(r_bP_Long = {0.6, 0, 0}, innerRotationAxis = {0, -1, 0}) annotation(Placement(visible = true, transformation(origin = {10, -86.894}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.Electronics.Battery.Battery battery(r_CM_Battery = {0, 0.1, 0}, m_Battery = 3, d_Battery = 5000, A_Battery = 0.25, c_d_Battery = 1, C_Battery = 18) annotation(Placement(visible = true, transformation(origin = {12.761, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.FrameBody.TopFrame.TopPiece topPiece3 annotation(Placement(visible = true, transformation(origin = {45, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.Propeller.Examples.BasicProp basicProp5 annotation(Placement(visible = true, transformation(origin = {45, -60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanConstant booleanConstant(k = false) annotation(Placement(visible = true, transformation(origin = {-180, 75}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(leftSide.frame_frontU, topPiece.frame_SF) annotation(Line(visible = true, origin = {-66.113, 38.238}, points = {{-8.071, -29.238}, {-8.071, 14.619}, {16.143, 14.619}}, color = {95, 95, 95}));
  connect(basicProp.frame_b, topPiece.frame_propA) annotation(Line(visible = true, origin = {-40, 74.077}, points = {{0, 4.246}, {0, -4.246}}, color = {95, 95, 95}));
  connect(leftSide.frame_propV, basicProp1.frame_b) annotation(Line(visible = true, origin = {-63.333, 3.914}, points = {{-6.667, -3.914}, {3.333, -3.914}, {3.333, 7.828}}, color = {95, 95, 95}));
  connect(leftSide.lumenU, lumen.frame_a) annotation(Line(visible = true, origin = {-71.11, 20.689}, points = {{1.11, -12.787}, {6.39, -12.787}, {6.39, 12.787}, {-13.89, 12.787}}, color = {95, 95, 95}));
  connect(leftSide.lumenL, lumen1.frame_a) annotation(Line(visible = true, origin = {-71.071, -19.04}, points = {{1.227, 10.96}, {6.351, 10.96}, {6.351, -10.96}, {-13.929, -10.96}}, color = {95, 95, 95}));
  connect(leftSide.frame_backU, topPiece1.frame_SF) annotation(Line(visible = true, origin = {-67.303, -18.238}, points = {{-7.097, 9.238}, {-7.097, -4.619}, {14.194, -4.619}}, color = {95, 95, 95}));
  connect(topPiece1.frame_propA, basicProp2.frame_b) annotation(Line(visible = true, origin = {-43.139, -44.916}, points = {{0, 5.084}, {0, -5.084}}, color = {95, 95, 95}));
  connect(topPiece.frame_EC, eEnclosure.frame_lf) annotation(Line(visible = true, origin = {-29.762, 39.952}, points = {{-0.208, 12.905}, {3.462, 12.905}, {3.462, 7.048}, {-3.238, 7.048}, {-3.238, -19.952}, {-0.238, -19.952}}, color = {95, 95, 95}));
  connect(topPiece1.frame_EC, eEnclosure.frame_lb) annotation(Line(visible = true, origin = {-31.331, -10.619}, points = {{-1.777, -12.238}, {1.892, -12.238}, {1.892, -6.381}, {-1.669, -6.381}, {-1.669, 18.619}, {1.331, 18.619}}, color = {95, 95, 95}));
  connect(basicProp3.frame_b, topPiece2.frame_propA) annotation(Line(visible = true, origin = {20, 74.916}, points = {{0, 5.084}, {0, -5.084}}, color = {95, 95, 95}));
  connect(topPiece2.frame_EC, rightSide.frame_backU) annotation(Line(visible = true, origin = {56.277, 38.238}, points = {{-26.247, 14.619}, {13.123, 14.619}, {13.123, -29.238}}, color = {95, 95, 95}));
  connect(topPiece2.frame_SF, eEnclosure.frame_rf) annotation(Line(visible = true, origin = {-3.492, 36.429}, points = {{13.523, 16.429}, {-3.508, 16.429}, {-3.508, -16.429}, {-6.508, -16.429}}, color = {95, 95, 95}));
  connect(lumen2.frame_a, rightSide.lumenL) annotation(Line(visible = true, origin = {65.399, 19.04}, points = {{11.913, 10.96}, {-5.679, 10.96}, {-5.679, -10.96}, {-0.555, -10.96}}, color = {95, 95, 95}));
  connect(rightSide.lumenU, lumen3.frame_a) annotation(Line(visible = true, origin = {65.438, -18.951}, points = {{-0.438, 11.049}, {-5.718, 11.049}, {-5.718, -11.049}, {11.874, -11.049}}, color = {95, 95, 95}));
  connect(rightSide.frame_propV, basicProp4.frame_b) annotation(Line(visible = true, origin = {52.727, 1.667}, points = {{12.273, -1.667}, {-6.137, -1.667}, {-6.137, 3.333}}, color = {95, 95, 95}));
  connect(leftSide.frame_frontL, bottomPlate.frame_lf) annotation(Line(visible = true, origin = {-80.483, -25.22}, points = {{-5.74, 32.62}, {-5.74, 38.22}, {-34.517, 38.22}, {-34.517, -54.531}, {80.513, -54.531}}, color = {95, 95, 95}));
  connect(leftSide.frame_backL, bottomPlate.frame_lb) annotation(Line(visible = true, origin = {-78.467, -46.111}, points = {{-7.716, 37.711}, {-7.716, 29.546}, {-31.533, 29.546}, {-31.533, -48.402}, {78.497, -48.402}}, color = {95, 95, 95}));
  connect(rightSide.frame_frontL, bottomPlate.frame_rf) annotation(Line(visible = true, origin = {79.703, -38.58}, points = {{1.52, 31.18}, {1.52, 25.58}, {28.316, 25.58}, {28.316, -41.171}, {-59.673, -41.171}}, color = {95, 95, 95}));
  connect(rightSide.frame_backL, bottomPlate.frame_rb) annotation(Line(visible = true, origin = {83.684, -30.918}, points = {{-2.501, 39.318}, {-2.501, 43.918}, {34.343, 43.918}, {34.343, -63.576}, {-63.684, -63.576}}, color = {95, 95, 95}));
  connect(battery.frame_a, bottomPlate.frame_bat) annotation(Line(visible = true, origin = {4.457, -64.936}, points = {{-1.696, 14.936}, {-4.696, 14.936}, {-4.696, -8.958}, {5.543, -8.958}, {5.543, -11.958}}, color = {95, 95, 95}));
  connect(battery.pin_p, eEnclosure.pin_p) annotation(Line(visible = true, origin = {-13.497, -8.157}, points = {{16.258, -31.843}, {16.258, -28.843}, {18.497, -28.843}, {18.497, 11.608}, {-26.503, 11.608}, {-26.503, 33.157}, {-16.503, 33.157}}, color = {10, 90, 224}));
  connect(eEnclosure.pin_n, lumen.pin_p) annotation(Line(visible = true, origin = {-27.25, 34.238}, points = {{17.25, -9.238}, {20.25, -9.238}, {20.25, 9.238}, {-57.75, 9.238}}, color = {10, 90, 224}));
  connect(lumen.pin_n, lumen1.pin_p) annotation(Line(visible = true, origin = {-91.7, 11.385}, points = {{-13.3, 32.09}, {-13.3, 35.34}, {9.95, 35.34}, {9.95, -51.385}, {6.7, -51.385}}, color = {10, 90, 224}));
  connect(lumen1.pin_n, lumen3.pin_p) annotation(Line(visible = true, origin = {-49.453, -33.3}, points = {{-55.547, -6.7}, {-55.547, -9.95}, {-7.835, -9.95}, {-7.835, 13.3}, {126.765, 13.3}}, color = {10, 90, 224}));
  connect(lumen3.pin_n, lumen2.pin_p) annotation(Line(visible = true, origin = {84.012, 5.3}, points = {{13.3, -25.3}, {13.3, -22.05}, {-9.95, -22.05}, {-9.95, 34.7}, {-6.7, 34.7}}, color = {10, 90, 224}));
  connect(lumen2.pin_n, battery.pin_n) annotation(Line(visible = true, origin = {53.781, 9.3}, points = {{43.531, 30.7}, {43.531, 33.95}, {-28.02, 33.95}, {-28.02, -49.3}, {-31.02, -49.3}}, color = {10, 90, 224}));
  connect(eEnclosure.pin_n, basicProp.pin_p) annotation(Line(visible = true, origin = {-24.8, 70.284}, points = {{14.8, -45.284}, {17.8, -45.284}, {17.8, 31.264}, {-25.2, 31.264}, {-25.2, 28.039}}, color = {10, 90, 224}));
  connect(basicProp.pin_n, battery.pin_n) annotation(Line(visible = true, origin = {11.071, 29.162}, points = {{-41.071, 69.162}, {14.69, 69.162}, {14.69, -69.162}, {11.69, -69.162}}, color = {10, 90, 224}));
  connect(eEnclosure.pin_n, basicProp3.pin_p) annotation(Line(visible = true, origin = {-0.8, 71.29}, points = {{-9.2, -46.29}, {-6.2, -46.29}, {-6.2, 31.935}, {10.8, 31.935}, {10.8, 28.71}}, color = {10, 90, 224}));
  connect(basicProp3.pin_n, battery.pin_n) annotation(Line(visible = true, origin = {29.803, 30}, points = {{0.197, 70}, {3.422, 70}, {3.422, -70}, {-7.042, -70}}, color = {10, 90, 224}));
  connect(eEnclosure.pin_n, basicProp1.pin_p) annotation(Line(visible = true, origin = {-40.075, 30.57}, points = {{30.075, -5.57}, {33.075, -5.57}, {33.075, 4.397}, {-33.15, 4.397}, {-33.15, 1.172}, {-29.925, 1.172}}, color = {10, 90, 224}));
  connect(basicProp1.pin_n, battery.pin_n) annotation(Line(visible = true, origin = {6.071, -4.129}, points = {{-56.071, 35.871}, {19.69, 35.871}, {19.69, -35.871}, {16.69, -35.871}}, color = {10, 90, 224}));
  connect(eEnclosure.pin_n, basicProp4.pin_p) annotation(Line(visible = true, origin = {9.836, 26.29}, points = {{-19.836, -1.29}, {-16.836, -1.29}, {-16.836, 1.935}, {26.754, 1.935}, {26.754, -1.29}}, color = {10, 90, 224}));
  connect(basicProp4.pin_n, battery.pin_n) annotation(Line(visible = true, origin = {49.745, -7.5}, points = {{6.845, 32.5}, {10.07, 32.5}, {10.07, -32.5}, {-26.984, -32.5}}, color = {10, 90, 224}));
  connect(eEnclosure.pin_n, basicProp2.pin_p) annotation(Line(visible = true, origin = {-31.644, -14.333}, points = {{21.644, 39.333}, {24.644, 39.333}, {24.644, 16.333}, {-24.719, 16.333}, {-24.719, -55.667}, {-21.494, -55.667}}, color = {10, 90, 224}));
  connect(basicProp2.pin_n, battery.pin_n) annotation(Line(visible = true, origin = {1.601, -59.29}, points = {{-34.74, -10.71}, {-34.74, -13.935}, {24.16, -13.935}, {24.16, 19.29}, {21.16, 19.29}}, color = {10, 90, 224}));
  connect(topPiece3.frame_EC, rightSide.frame_frontU) annotation(Line(visible = true, origin = {64.466, -18.238}, points = {{-9.436, -4.619}, {4.718, -4.619}, {4.718, 9.238}}, color = {95, 95, 95}));
  connect(topPiece3.frame_SF, eEnclosure.frame_rb) annotation(Line(visible = true, origin = {2.758, -7.429}, points = {{32.273, -15.429}, {-9.758, -15.429}, {-9.758, 15.429}, {-12.758, 15.429}}, color = {95, 95, 95}));
  connect(basicProp5.frame_b, topPiece3.frame_propA) annotation(Line(visible = true, origin = {45, -44.916}, points = {{0, -5.084}, {0, 5.084}}, color = {95, 95, 95}));
  connect(eEnclosure.pin_n, basicProp5.pin_p) annotation(Line(visible = true, origin = {9.2, -33.29}, points = {{-19.2, 58.29}, {-16.2, 58.29}, {-16.2, -39.935}, {25.8, -39.935}, {25.8, -36.71}}, color = {10, 90, 224}));
  connect(basicProp5.pin_n, battery.pin_n) annotation(Line(visible = true, origin = {48.553, -55}, points = {{6.447, -15}, {9.672, -15}, {9.672, 15}, {-25.792, 15}}, color = {10, 90, 224}));
  connect(booleanConstant.y, basicProp.u) annotation(Line(visible = true, origin = {-116.2, 90.284}, points = {{-52.8, -15.284}, {-49.8, -15.284}, {-49.8, 11.264}, {76.2, 11.264}, {76.2, 8.039}}, color = {190, 52, 178}));
  connect(booleanConstant.y, basicProp3.u) annotation(Line(visible = true, origin = {-92.2, 91.29}, points = {{-76.8, -16.29}, {-73.8, -16.29}, {-73.8, 11.935}, {112.2, 11.935}, {112.2, 8.71}}, color = {190, 52, 178}));
  connect(booleanConstant.y, basicProp1.u) annotation(Line(visible = true, origin = {-96.333, 60.581}, points = {{-72.667, 14.419}, {36.333, 14.419}, {36.333, -28.839}}, color = {190, 52, 178}));
  connect(booleanConstant.y, basicProp4.u) annotation(Line(visible = true, origin = {-25.273, 58.333}, points = {{-143.727, 16.667}, {71.863, 16.667}, {71.863, -33.333}}, color = {190, 52, 178}));
  connect(booleanConstant.y, basicProp2.u) annotation(Line(visible = true, origin = {-117.456, -13.29}, points = {{-51.544, 88.29}, {-48.544, 88.29}, {-48.544, -59.935}, {74.317, -59.935}, {74.317, -56.71}}, color = {190, 52, 178}));
  connect(booleanConstant.y, basicProp5.u) annotation(Line(visible = true, origin = {-82.2, -13.29}, points = {{-86.8, 88.29}, {-83.8, 88.29}, {-83.8, -59.935}, {127.2, -59.935}, {127.2, -56.71}}, color = {190, 52, 178}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end BasicBlueROV2;
