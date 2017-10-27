within ROVm.FrameBody.BottomFrame;

model BlueROV2
  import SI = Modelica.SIunits;
  RBodyInFluid.Parts.BasicBody ballast1(m = m_ballast, r_CM = r_CM_ballast, density = d_ballast, c_d = c_d_ballast, A = A_Ballast) annotation(Placement(visible = true, transformation(origin = {-85, 32.929}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody ballast2(m = m_ballast, r_CM = r_CM_ballast, density = d_ballast, c_d = c_d_ballast, A = A_Ballast) annotation(Placement(visible = true, transformation(origin = {75, 32.929}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody ballast3(m = m_ballast, r_CM = r_CM_ballast, density = d_ballast, c_d = c_d_ballast, A = A_Ballast) annotation(Placement(visible = true, transformation(origin = {115, 95}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody ballast4(m = m_ballast, r_CM = r_CM_ballast, density = d_ballast, c_d = c_d_ballast, A = A_Ballast) annotation(Placement(visible = true, transformation(origin = {83.34, -45}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody ballast5(m = m_ballast, r_CM = r_CM_ballast, density = d_ballast, c_d = c_d_ballast, A = A_Ballast) annotation(Placement(visible = true, transformation(origin = {-90, -45}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_lf "Left front connection to side plates, as viewed from the back" annotation(Placement(visible = true, transformation(origin = {-148.052, 75}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-99.698, 71.429}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_lb "Left back connection to side plate, as viewed from the back" annotation(Placement(visible = true, transformation(origin = {-148.052, -80}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-99.698, -76.19}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_rf "Right front connection to side plate, as viewed from the back" annotation(Placement(visible = true, transformation(origin = {150, 85}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100.302, 71.429}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_rb "Right back connection to side plate, as viewed from the back" annotation(Placement(visible = true, transformation(origin = {148.052, -82.481}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -76}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_bat "Connection to the battery enclosure" annotation(Placement(visible = true, transformation(origin = {0, 105}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = ballastPosition1) annotation(Placement(visible = true, transformation(origin = {-36.533, 32.929}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation2(r = ballastPosition2) annotation(Placement(visible = true, transformation(origin = {25, 32.929}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation3(r = ballastPosition3) annotation(Placement(visible = true, transformation(origin = {65, 94.889}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation4(r = ballastPosition4) annotation(Placement(visible = true, transformation(origin = {33.34, -45}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation5(r = ballastPosition5) annotation(Placement(visible = true, transformation(origin = {-55, -45}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBodyShape bottomPlateU(density = d_plate, r = r_bP_Long, r_CM = r_CM_bP_Long, m = m_bP_Long, A = A_bP_Long, c_d = c_d_bP_Long) annotation(Placement(visible = true, transformation(origin = {-5, 66.951}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBodyShape bottomPlateB(density = d_plate, r = r_bP_Long, r_CM = r_CM_bP_Long, m = m_bP_Long, A = A_bP_Long, c_d = c_d_bP_Long) annotation(Placement(visible = true, transformation(origin = {-16.565, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBodyShape bottomPlateC(density = d_plate, r = r_bP_Short, r_CM = r_CM_bP_Short, m = m_bP_Short, A = A_bP_Short, c_d = c_d_bP_Short) annotation(Placement(visible = true, transformation(origin = {32.785, 95}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter SI.Density d_plate = 2700;
  parameter SI.Length r_bP_Long = {0.6, 0, 0};
  parameter SI.Length r_CM_bP_Long = {0.3, 0, 0};
  parameter SI.Mass m_bP_Long = 0.25;
  parameter SI.Area A_bP_Long = 0.0009;
  parameter SI.DimensionlessRatio c_d_bP_Long = 1;
  parameter SI.DimensionlessRatio c_d_bP_Short = 1;
protected
  // Ballast physical characeristics
  parameter Modelica.SIunits.Mass m_ballast = 200;
  parameter Modelica.SIunits.Length r_CM_ballast[3] = {0.254, 0, 0};
  parameter Modelica.SIunits.Density d_ballast = 7700;
  parameter Modelica.SIunits.DimensionlessRatio c_d_ballast = 1;
  parameter Modelica.SIunits.Area A_Ballast = 0.000375;
  // Ballast positioning
  parameter SI.Length ballastPosition1[3] = {0.15, 0, 0};
  parameter SI.Length ballastPosition2[3] = {0.75, 0, 0};
  parameter SI.Length ballastPosition3[3] = {0.10, 0, 0};
  parameter SI.Length ballastPosition4[3] = {0.75, 0, 0};
  parameter SI.Length ballastPosition5[3] = {0.15, 0, 0};
  // Inner connection (bottomPlateC) characteristics
  parameter SI.Area A_bP_Short = A_bP_Long / 3;
  parameter SI.Mass m_bP_Short = m_bP_Long / 3;
  parameter SI.Length r_CM_bP_Short = r_CM_bP_Long / 3;
  parameter SI.Length r_bP_Short = r_bP_Long / 3;
equation
  connect(fixedTranslation1.frame_b, ballast1.frame_a) annotation(Line(visible = true, origin = {-60.766, 32.929}, points = {{14.234, 0}, {-14.234, -0}}, color = {95, 95, 95}));
  connect(fixedTranslation2.frame_b, ballast2.frame_a) annotation(Line(visible = true, origin = {50, 32.929}, points = {{-15, 0}, {15, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation4.frame_b, ballast4.frame_a) annotation(Line(visible = true, origin = {58.34, -45}, points = {{-15, 0}, {15, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation5.frame_b, ballast5.frame_a) annotation(Line(visible = true, origin = {-72.5, -45}, points = {{7.5, 0}, {-7.5, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation3.frame_b, ballast3.frame_a) annotation(Line(visible = true, origin = {95.994, 94.944}, points = {{-20.994, -0.056}, {5.994, -0.056}, {5.994, 0.056}, {9.006, 0.056}}, color = {95, 95, 95}));
  connect(fixedTranslation3.frame_a, bottomPlateC.frame_b) annotation(Line(visible = true, origin = {47.395, 94.944}, points = {{7.605, -0.056}, {-1.497, -0.056}, {-1.497, 0.056}, {-4.61, 0.056}}, color = {95, 95, 95}));
  connect(bottomPlateC.frame_a, frame_bat) annotation(Line(visible = true, origin = {15.196, 100}, points = {{7.589, -5}, {3.804, -5}, {3.804, 5}, {-15.196, 5}}, color = {95, 95, 95}));
  connect(bottomPlateU.frame_b, frame_rf) annotation(Line(visible = true, origin = {104.25, 75.976}, points = {{-99.25, -9.024}, {26.75, -9.024}, {26.75, 9.024}, {45.75, 9.024}}, color = {95, 95, 95}));
  connect(bottomPlateU.frame_a, frame_lf) annotation(Line(visible = true, origin = {-105.289, 70.976}, points = {{90.289, -4.024}, {-23.763, -4.024}, {-23.763, 4.024}, {-42.763, 4.024}}, color = {95, 95, 95}));
  connect(fixedTranslation1.frame_a, bottomPlateU.frame_a) annotation(Line(visible = true, origin = {-19.389, 49.94}, points = {{-7.143, -17.011}, {1.377, -17.011}, {1.377, 17.011}, {4.389, 17.011}}, color = {95, 95, 95}));
  connect(fixedTranslation2.frame_a, bottomPlateU.frame_a) annotation(Line(visible = true, origin = {-9.006, 49.94}, points = {{24.006, -17.011}, {-9.006, -17.011}, {-9.006, 17.011}, {-5.994, 17.011}}, color = {95, 95, 95}));
  connect(fixedTranslation5.frame_a, bottomPlateB.frame_a) annotation(Line(visible = true, origin = {-32.68, -62.5}, points = {{-12.32, 17.5}, {3.102, 17.5}, {3.102, -17.5}, {6.115, -17.5}}, color = {95, 95, 95}));
  connect(fixedTranslation4.frame_a, bottomPlateB.frame_a) annotation(Line(visible = true, origin = {-15.595, -62.5}, points = {{38.935, 17.5}, {-13.982, 17.5}, {-13.982, -17.5}, {-10.97, -17.5}}, color = {95, 95, 95}));
  connect(bottomPlateB.frame_a, frame_lb) annotation(Line(visible = true, origin = {-87.309, -80}, points = {{60.743, 0}, {-60.743, 0}}, color = {95, 95, 95}));
  connect(bottomPlateB.frame_b, frame_rb) annotation(Line(visible = true, origin = {99.898, -81.24}, points = {{-106.463, 1.24}, {29.154, 1.24}, {29.154, -1.24}, {48.154, -1.24}}, color = {95, 95, 95}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end BlueROV2;
