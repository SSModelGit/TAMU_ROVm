within UWBody.Examples.Loops.Utilities;

model Cylinder "Cylinder with rod and crank of a combustion engine"
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter SI.Length cylinderTopPosition = 0.42 "Length from crank shaft to end of cylinder.";
  parameter SI.Length pistonLength = 0.1 "Length of cylinder";
  parameter SI.Length rodLength = 0.2 "Length of rod";
  parameter SI.Length crankLength = 0.2 "Length of crank shaft in x direction";
  parameter SI.Length crankPinOffset = 0.1 "Offset of crank pin from center axis";
  parameter SI.Length crankPinLength = 0.1 "Offset of crank pin from center axis";
  parameter Modelica.SIunits.Angle cylinderInclination = 0 "Inclination of cylinder";
  parameter Modelica.SIunits.Angle crankAngleOffset = 0 "Offset for crank angle";
  parameter SI.Length cylinderLength = cylinderTopPosition - (pistonLength + rodLength - crankPinOffset) "Maximum length of cylinder volume";
  UWBody.Parts.BodyCylinder Piston(diameter = 0.1, r = {0, pistonLength, 0}, color = {180, 180, 180}, animation = animation) annotation(Placement(transformation(origin = {14.5, 69.5}, extent = {{10.5, -30.5}, {-10.5, 30.5}}, rotation = 270)));
  UWBody.Parts.BodyBox Rod(widthDirection = {1, 0, 0}, height = 0.06, color = {0, 0, 200}, width = 0.02, r_shape = {0, -0.02, 0}, r = {0, rodLength, 0}, animation = animation) annotation(Placement(transformation(origin = {14, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  UWBody.Joints.Revolute B2(n = {1, 0, 0}, cylinderLength = 0.02, animation = animation, cylinderDiameter = 0.055) annotation(Placement(transformation(extent = {{4, 25}, {24, 45}})));
  UWBody.Parts.BodyBox Crank4(height = 0.05, widthDirection = {1, 0, 0}, width = 0.02, r = {0, -crankPinOffset, 0}, animation = animation) annotation(Placement(transformation(origin = {40.5, -74}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  UWBody.Parts.BodyCylinder Crank3(r_shape = {-0.01, 0, 0}, length = 0.12, diameter = 0.03, r = {crankPinLength, 0, 0}, color = {180, 180, 180}, animation = animation) annotation(Placement(transformation(extent = {{4.5, -60}, {24.5, -40}})));
  UWBody.Parts.BodyCylinder Crank1(diameter = 0.05, r_shape = {-0.01, 0, 0}, length = 0.12, r = {crankLength - crankPinLength, 0, 0}, color = {180, 180, 180}, animation = animation) annotation(Placement(transformation(extent = {{-50, -100}, {-30, -80}})));
  UWBody.Parts.BodyBox Crank2(height = 0.05, widthDirection = {1, 0, 0}, width = 0.02, r = {0, crankPinOffset, 0}, animation = animation) annotation(Placement(transformation(origin = {-10, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Joints.RevolutePlanarLoopConstraint B1(n = {1, 0, 0}, cylinderLength = 0.02, animation = animation, cylinderDiameter = 0.055) annotation(Placement(transformation(extent = {{4, -27}, {24, -7}})));
  UWBody.Parts.FixedTranslation Mid(r = {crankPinLength / 2, 0, 0}, animation = false) annotation(Placement(transformation(extent = {{-6, -46}, {14, -26}})));
  UWBody.Joints.Prismatic Cylinder(useAxisFlange = true, s(start = -0.3), n = {0, -1, 0}, boxWidth = 0.02) annotation(Placement(transformation(origin = {14, 99}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
  UWBody.Parts.FixedTranslation Mounting(r = {crankLength, 0, 0}, animation = false) annotation(Placement(transformation(extent = {{0, 120}, {20, 140}})));
  UWBody.Parts.FixedRotation CylinderInclination(r = {crankLength - crankPinLength / 2, 0, 0}, n_y = {0, cos(cylinderInclination), sin(cylinderInclination)}, animation = false, rotationType = UWBody.Types.RotationTypes.TwoAxesVectors) annotation(Placement(transformation(extent = {{-60, 30}, {-40, 50}})));
  UWBody.Parts.FixedRotation CrankAngle1(n_y = {0, cos(crankAngleOffset), sin(crankAngleOffset)}, animation = false, rotationType = UWBody.Types.RotationTypes.TwoAxesVectors) annotation(Placement(transformation(extent = {{-90, -100}, {-70, -80}})));
  UWBody.Parts.FixedRotation CrankAngle2(n_y = {0, cos(-crankAngleOffset), sin(-crankAngleOffset)}, animation = false, rotationType = UWBody.Types.RotationTypes.TwoAxesVectors) annotation(Placement(transformation(extent = {{60, -100}, {80, -80}})));
  UWBody.Parts.FixedTranslation CylinderTop(r = {0, cylinderTopPosition, 0}, animation = false) annotation(Placement(transformation(origin = {-30, 71}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  GasForce gasForce(L = cylinderLength, d = 0.1) annotation(Placement(transformation(origin = {50, 107}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Interfaces.Frame_a cylinder_a annotation(Placement(transformation(extent = {{-116, 114}, {-84, 146}})));
  Interfaces.Frame_a cylinder_b annotation(Placement(transformation(extent = {{84, 114}, {116, 146}})));
  Interfaces.Frame_a crank_a annotation(Placement(transformation(extent = {{-116, -106}, {-84, -74}})));
  Interfaces.Frame_a crank_b annotation(Placement(transformation(extent = {{84, -106}, {116, -74}})));
equation
  connect(B1.frame_a, Mid.frame_b) annotation(Line(points = {{4, -17}, {-6, -17}, {-6, -29}, {22, -29}, {22, -36}, {14, -36}}, color = {95, 95, 95}, thickness = 0.5));
  connect(Rod.frame_a, B1.frame_b) annotation(Line(points = {{14, -2}, {14, -9}, {30, -9}, {30, -17}, {24, -17}}, color = {95, 95, 95}, thickness = 0.5));
  connect(Cylinder.frame_b, Piston.frame_b) annotation(Line(points = {{14, 89}, {14, 80}, {14.5, 80}}, color = {95, 95, 95}, thickness = 0.5));
  connect(Crank1.frame_a, CrankAngle1.frame_b) annotation(Line(points = {{-50, -90}, {-70, -90}}, color = {95, 95, 95}, thickness = 0.5));
  connect(B2.frame_a, Piston.frame_a) annotation(Line(points = {{4, 35}, {-6, 35}, {-6, 49}, {14.5, 49}, {14.5, 59}}, color = {95, 95, 95}, thickness = 0.5));
  connect(Rod.frame_b, B2.frame_b) annotation(Line(points = {{14, 18}, {14, 23}, {32, 23}, {32, 35}, {24, 35}}, color = {95, 95, 95}, thickness = 0.5));
  connect(Crank4.frame_b, CrankAngle2.frame_a) annotation(Line(points = {{40.5, -84}, {40.5, -90}, {60, -90}}, color = {95, 95, 95}, thickness = 0.5));
  connect(Cylinder.support, gasForce.flange_b) annotation(Line(points = {{20, 103}, {34, 103}, {34, 117}, {50, 117}}, color = {0, 191, 0}));
  connect(Cylinder.axis, gasForce.flange_a) annotation(Line(points = {{20, 91}, {50, 91}, {50, 97}}, color = {0, 191, 0}));
  connect(CylinderInclination.frame_b, CylinderTop.frame_a) annotation(Line(points = {{-40, 40}, {-30, 40}, {-30, 61}}));
  connect(Crank1.frame_b, Crank2.frame_a) annotation(Line(points = {{-30, -90}, {-10, -90}, {-10, -86}}, color = {95, 95, 95}, thickness = 0.5));
  connect(Crank3.frame_b, Crank4.frame_a) annotation(Line(points = {{24.5, -50}, {40.5, -50}, {40.5, -64}}, color = {95, 95, 95}, thickness = 0.5));
  connect(Crank3.frame_a, Crank2.frame_b) annotation(Line(points = {{4.5, -50}, {-10, -50}, {-10, -66}}, color = {95, 95, 95}, thickness = 0.5));
  connect(Crank2.frame_b, Mid.frame_a) annotation(Line(points = {{-10, -66}, {-10, -36}, {-6, -36}}, color = {95, 95, 95}, thickness = 0.5));
  connect(CylinderTop.frame_b, Cylinder.frame_a) annotation(Line(points = {{-30, 81}, {-30, 120}, {14, 120}, {14, 109}}, color = {95, 95, 95}, thickness = 0.5));
  connect(CylinderInclination.frame_a, cylinder_a) annotation(Line(points = {{-60, 40}, {-80, 40}, {-80, 130}, {-100, 130}}, color = {95, 95, 95}, thickness = 0.5));
  connect(Mounting.frame_a, cylinder_a) annotation(Line(points = {{0, 130}, {-100, 130}}, color = {95, 95, 95}, thickness = 0.5));
  connect(Mounting.frame_b, cylinder_b) annotation(Line(points = {{20, 130}, {100, 130}}, color = {95, 95, 95}, thickness = 0.5));
  connect(CrankAngle1.frame_a, crank_a) annotation(Line(points = {{-90, -90}, {-100, -90}}, color = {95, 95, 95}, thickness = 0.5));
  connect(CrankAngle2.frame_b, crank_b) annotation(Line(points = {{80, -90}, {100, -90}}, color = {95, 95, 95}, thickness = 0.5));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -150}, {100, 150}}, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {0, 32}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.VerticalCylinder, points = {{-58, 58}, {58, 58}, {58, -22}, {50, -22}, {40, -14}, {-40, -14}, {-50, -22}, {-58, -22}}), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, points = {{-60, -50}, {-60, 100}, {60, 100}, {60, -52}, {100, -52}, {100, 150}, {-100, 150}, {-100, -50}, {-60, -50}}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, extent = {{-60, 75}, {60, 81}}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, extent = {{-60, 61}, {60, 67}}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, extent = {{-60, 49}, {60, 55}}), Ellipse(visible = true, lineColor = {192, 192, 192}, extent = {{-40, -129}, {40, -49}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -165}, {150, -125}}, textString = "%name"), Line(visible = true, points = {{-100, -90}, {100, -91}}, thickness = 0.5), Ellipse(visible = true, origin = {0, 34}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, extent = {{-10, -10}, {10, 10}}), Line(visible = true, points = {{0, -90}, {26, -58}, {0, 34}})}), Documentation(info = "<html>
<p>
Cylinder with rod and crank of a combustion engine.
Used as submodel in <a href=\"modelica://UWBody.Examples.Loops.EngineV6\">Loops.EngineV6</a>.
</p>
</html>"));
end Cylinder;
