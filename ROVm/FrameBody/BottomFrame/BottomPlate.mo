within ROVm.FrameBody.BottomFrame;

model BottomPlate
  import SI = Modelica.SIunits;
  RBodyInFluid.Parts.BasicBody ballast1(m = m_ballast, r_CM = r_CM_ballast, density = d_ballast, c_d = c_d_ballast, A = A_Ballast, animation = animation) annotation(Placement(visible = true, transformation(origin = {-85, 32.929}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody ballast2(m = m_ballast, r_CM = r_CM_ballast, density = d_ballast, c_d = c_d_ballast, A = A_Ballast, animation = animation) annotation(Placement(visible = true, transformation(origin = {75, 32.929}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody ballast3(m = m_ballast, r_CM = r_CM_ballast, density = d_ballast, c_d = c_d_ballast, A = A_Ballast, animation = animation) annotation(Placement(visible = true, transformation(origin = {115, 95}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody ballast4(m = m_ballast, r_CM = r_CM_ballast, density = d_ballast, c_d = c_d_ballast, A = A_Ballast, animation = animation) annotation(Placement(visible = true, transformation(origin = {83.34, -45}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody ballast5(m = m_ballast, r_CM = r_CM_ballast, density = d_ballast, c_d = c_d_ballast, A = A_Ballast, animation = animation) annotation(Placement(visible = true, transformation(origin = {-90, -45}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_lf "Left front connection to side plates, as viewed from the back" annotation(Placement(visible = true, transformation(origin = {-148.052, 75}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-99.698, 71.429}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_lb "Left back connection to side plate, as viewed from the back" annotation(Placement(visible = true, transformation(origin = {-148.052, -80}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-99.698, -76.19}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_rf "Right front connection to side plate, as viewed from the back" annotation(Placement(visible = true, transformation(origin = {150, 85}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100.302, 71.429}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_rb "Right back connection to side plate, as viewed from the back" annotation(Placement(visible = true, transformation(origin = {148.052, -82.481}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -76}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_bat "Connection to the battery enclosure" annotation(Placement(visible = true, transformation(origin = {0, 105}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = ballastPosition1, animation = animationFT) annotation(Placement(visible = true, transformation(origin = {-36.533, 32.929}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation2(r = ballastPosition2, animation = animationFT) annotation(Placement(visible = true, transformation(origin = {25, 32.929}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation3(r = ballastPosition3, animation = animationFT) annotation(Placement(visible = true, transformation(origin = {85, 95}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation4(r = ballastPosition4, animation = animationFT) annotation(Placement(visible = true, transformation(origin = {33.34, -45}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation5(r = ballastPosition5, animation = animationFT) annotation(Placement(visible = true, transformation(origin = {-55, -45}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBodyShape bottomPlateU(density = d_plate, r = r_bP_Long, r_CM = r_CM_bP_Long, m = m_bP_Long, A = A_bP_Long, c_d = c_d_bP_Long, animation = animation) annotation(Placement(visible = true, transformation(origin = {-5, 66.951}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBodyShape bottomPlateB(density = d_plate, r = r_bP_Long, r_CM = r_CM_bP_Long, m = m_bP_Long, A = A_bP_Long, c_d = c_d_bP_Long, animation = animation) annotation(Placement(visible = true, transformation(origin = {-16.565, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBodyShape bottomPlateC(density = d_plate, r = r_bP_Short, r_CM = r_CM_bP_Short, m = m_bP_Short, A = A_bP_Short, c_d = c_d_bP_Short, animation = animation) annotation(Placement(visible = true, transformation(origin = {52.785, 95.111}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter Boolean animation = true;
  parameter Boolean animationFT = true;
  parameter SI.Density d_plate = 2700;
  parameter SI.Length r_bP_Long[3] = {0, 0, 0.6};
  parameter SI.Length r_CM_bP_Long[3] = r_bP_Long / 2;
  parameter SI.Mass m_bP_Long = 0.25;
  parameter SI.Area A_bP_Long = 0.0009;
  parameter SI.DimensionlessRatio c_d_bP_Long = 1;
  parameter SI.DimensionlessRatio c_d_bP_Short = 1;
  parameter Real innerScaleFactor = 2;
  parameter SI.Length r_bP_Short[3] = {0.3, 0, 0};
  parameter Modelica.Mechanics.MultiBody.Types.Axis innerRotationAxis "Think wisely, grasshopper.";
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = r_bP_Short / 2, animation = animationFT) annotation(Placement(visible = true, transformation(origin = {25, 95}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation6(r = r_bP_Long / 2, animation = animationFT) annotation(Placement(visible = true, transformation(origin = {-67.641, 2.416}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation7(r = -1 * r_bP_Long / 2, animation = animationFT) annotation(Placement(visible = true, transformation(origin = {15, -20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
protected
  // Ballast physical characeristics
  parameter Modelica.SIunits.Mass m_ballast = 200;
  parameter Modelica.SIunits.Length r_CM_ballast[3] = {0.001, 0, 0};
  parameter Modelica.SIunits.Density d_ballast = 7700;
  parameter Modelica.SIunits.DimensionlessRatio c_d_ballast = 1;
  parameter Modelica.SIunits.Area A_Ballast = 0.000375;
  // Ballast positioning
  parameter SI.Length ballastPosition1[3] = r_bP_Long / 4;
  parameter SI.Length ballastPosition2[3] = r_bP_Long * 3 / 4;
  parameter SI.Length ballastPosition3[3] = r_bP_Short / 2;
  parameter SI.Length ballastPosition4[3] = r_bP_Long * 3 / 4;
  parameter SI.Length ballastPosition5[3] = r_bP_Long / 4;
  // Inner connection (bottomPlateC) characteristics
  parameter SI.Area A_bP_Short = A_bP_Long / innerScaleFactor;
  parameter SI.Mass m_bP_Short = m_bP_Long / innerScaleFactor;
  parameter SI.Length r_CM_bP_Short[3] = r_bP_Short / 2;
equation
  connect(fixedTranslation1.frame_b, ballast1.frame_a) annotation(Line(visible = true, origin = {-60.766, 32.929}, points = {{14.234, 0}, {-14.234, -0}}, color = {95, 95, 95}));
  connect(fixedTranslation2.frame_b, ballast2.frame_a) annotation(Line(visible = true, origin = {50, 32.929}, points = {{-15, 0}, {15, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation4.frame_b, ballast4.frame_a) annotation(Line(visible = true, origin = {58.34, -45}, points = {{-15, 0}, {15, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation5.frame_b, ballast5.frame_a) annotation(Line(visible = true, origin = {-72.5, -45}, points = {{7.5, 0}, {-7.5, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation3.frame_b, ballast3.frame_a) annotation(Line(visible = true, origin = {100, 95}, points = {{-5, 0}, {5, 0}}, color = {95, 95, 95}));
  connect(bottomPlateU.frame_b, frame_rf) annotation(Line(visible = true, origin = {104.25, 75.976}, points = {{-99.25, -9.025}, {26.75, -9.024}, {26.75, 9.024}, {45.75, 9.024}}, color = {95, 95, 95}));
  connect(bottomPlateU.frame_a, frame_lf) annotation(Line(visible = true, origin = {-105.289, 70.976}, points = {{90.289, -4.024}, {-23.763, -4.024}, {-23.763, 4.024}, {-42.763, 4.024}}, color = {95, 95, 95}));
  connect(fixedTranslation1.frame_a, bottomPlateU.frame_a) annotation(Line(visible = true, origin = {-19.389, 49.94}, points = {{-7.143, -17.011}, {1.377, -17.011}, {1.377, 17.011}, {4.389, 17.011}}, color = {95, 95, 95}));
  connect(fixedTranslation2.frame_a, bottomPlateU.frame_a) annotation(Line(visible = true, origin = {-9.006, 49.94}, points = {{24.006, -17.011}, {-9.006, -17.011}, {-9.006, 17.011}, {-5.994, 17.011}}, color = {95, 95, 95}));
  connect(fixedTranslation5.frame_a, bottomPlateB.frame_a) annotation(Line(visible = true, origin = {-32.68, -62.5}, points = {{-12.32, 17.5}, {3.102, 17.5}, {3.102, -17.5}, {6.115, -17.5}}, color = {95, 95, 95}));
  connect(fixedTranslation4.frame_a, bottomPlateB.frame_a) annotation(Line(visible = true, origin = {-15.595, -62.5}, points = {{38.935, 17.5}, {-13.982, 17.5}, {-13.982, -17.5}, {-10.97, -17.5}}, color = {95, 95, 95}));
  connect(bottomPlateB.frame_a, frame_lb) annotation(Line(visible = true, origin = {-87.309, -80}, points = {{60.743, 0}, {-60.743, 0}}, color = {95, 95, 95}));
  connect(bottomPlateB.frame_b, frame_rb) annotation(Line(visible = true, origin = {99.898, -81.24}, points = {{-106.463, 1.24}, {29.154, 1.24}, {29.154, -1.24}, {48.154, -1.241}}, color = {95, 95, 95}));
  connect(fixedTranslation3.frame_a, bottomPlateC.frame_a) annotation(Line(visible = true, origin = {56.888, 90.736}, points = {{18.112, 4.264}, {15.112, 4.264}, {15.112, -8.638}, {-17.116, -8.638}, {-17.116, 4.375}, {-14.103, 4.375}}, color = {95, 95, 95}));
  connect(bottomPlateC.frame_a, fixedTranslation.frame_a) annotation(Line(visible = true, origin = {38.446, 95.055}, points = {{4.339, 0.056}, {-0.446, 0.056}, {-0.446, -0.055}, {-3.446, -0.055}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, frame_bat) annotation(Line(visible = true, origin = {10, 101.667}, points = {{5, -6.667}, {5, 3.333}, {-10, 3.333}}, color = {95, 95, 95}));
  connect(bottomPlateU.frame_a, fixedTranslation6.frame_a) annotation(Line(visible = true, origin = {-49.778, 29.789}, points = {{34.778, 37.162}, {31.765, 37.162}, {31.765, -9.789}, {-35.222, -9.789}, {-35.222, -27.373}, {-27.864, -27.373}}, color = {95, 95, 95}));
  connect(bottomPlateB.frame_b, fixedTranslation7.frame_a) annotation(Line(visible = true, origin = {54.609, -50}, points = {{-61.174, -30}, {45.391, -30}, {45.391, 30}, {-29.609, 30}}, color = {95, 95, 95}));
  connect(fixedTranslation7.frame_b, bottomPlateC.frame_a) annotation(Line(visible = true, origin = {18.869, 15.022}, points = {{-13.869, -35.022}, {-16.982, -35.022}, {-16.982, -5.022}, {23.916, -5.022}, {23.916, 80.089}}, color = {95, 95, 95}));
  connect(fixedTranslation6.frame_b, bottomPlateC.frame_b) annotation(Line(visible = true, origin = {32.863, 48.764}, points = {{-90.504, -46.348}, {30.291, -46.348}, {30.291, 46.348}, {29.922, 46.348}}, color = {95, 95, 95}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {-0.184, 58.875}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Vertical, points = {{-59.816, 18.43}, {-64.972, 13.276}, {-64.972, -11.756}, {-59.816, -18.875}, {60.184, -18.875}, {64.604, -11.756}, {64.604, 11.125}, {60.184, 18.43}}), Polygon(visible = true, origin = {0.184, -59.778}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Vertical, points = {{-59.816, 18.43}, {-64.972, 13.276}, {-64.972, -11.756}, {-59.816, -18.875}, {60.184, -18.875}, {64.604, -11.756}, {64.604, 11.125}, {60.184, 18.43}}), Polygon(visible = true, origin = {0, -0.399}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, points = {{-20, 30.399}, {-20, -29.601}, {-30, -41.198}, {30, -41.198}, {20, -29.601}, {20, 30.399}, {30, 40.399}, {-30, 40.399}}), Line(visible = true, origin = {-80.249, 64.42}, points = {{14.357, 0}, {-14.357, 0}}, color = {153, 153, 153}, thickness = 5), Line(visible = true, origin = {-80.433, -64.788}, points = {{14.909, 0}, {-14.909, 0}}, color = {153, 153, 153}, thickness = 5), Line(visible = true, origin = {-77.119, -78.409}, points = {{17.119, 0}, {-17.119, 0}}, color = {153, 153, 153}, thickness = 5), Line(visible = true, origin = {-77.671, 77.672}, points = {{17.671, 0}, {-17.671, 0}}, color = {153, 153, 153}, thickness = 5), Line(visible = true, origin = {78.591, 77.672}, points = {{-18.591, 0}, {18.591, 0}}, color = {153, 153, 153}, thickness = 5), Line(visible = true, origin = {79.881, 63.684}, points = {{-15.093, 0}, {15.093, 0}}, color = {153, 153, 153}, thickness = 5), Line(visible = true, origin = {78.039, -78.409}, points = {{-18.039, 0}, {18.039, 0}}, color = {153, 153, 153}, thickness = 5), Line(visible = true, origin = {80.065, -64.788}, points = {{-15.277, 0}, {15.277, 0}}, color = {153, 153, 153}, thickness = 5), Line(visible = true, origin = {0, 80.617}, points = {{0, -2.945}, {0, 2.945}}, thickness = 1)}), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end BottomPlate;
