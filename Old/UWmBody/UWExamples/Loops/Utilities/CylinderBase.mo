within UWmBody.UWExamples.Loops.Utilities;

model CylinderBase "One cylinder with analytic handling of kinematic loop"
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter SI.Length cylinderTopPosition = 0.42 "Length from crank shaft to end of cylinder.";
  parameter SI.Length crankLength = 0.14 "Length of crank shaft in x direction";
  parameter SI.Length crankPinOffset = 0.05 "Offset of crank pin from center axis";
  parameter SI.Length crankPinLength = 0.1 "Offset of crank pin from center axis";
  parameter Cv.NonSIunits.Angle_deg cylinderInclination = 0 "Inclination of cylinder";
  parameter Cv.NonSIunits.Angle_deg crankAngleOffset = 0 "Offset for crank angle";
  parameter SI.Length pistonLength = 0.1 "Length of cylinder" annotation(Dialog(group = "Piston"));
  parameter SI.Length pistonCenterOfMass = pistonLength / 2 "Distance from frame_a to center of mass of piston" annotation(Dialog(group = "Piston"));
  parameter SI.Mass pistonMass(min = 0) = 6 "Mass of piston" annotation(Dialog(group = "Piston"));
  parameter SI.Inertia pistonInertia_11(min = 0) = 0.0088 "Inertia 11 of piston with respect to center of mass frame, parallel to frame_a" annotation(Dialog(group = "Piston"));
  parameter SI.Inertia pistonInertia_22(min = 0) = 0.0076 "Inertia 22 of piston with respect to center of mass frame, parallel to frame_a" annotation(Dialog(group = "Piston"));
  parameter SI.Inertia pistonInertia_33(min = 0) = 0.0088 "Inertia 33 of piston with respect to center of mass frame, parallel to frame_a" annotation(Dialog(group = "Piston"));
  parameter SI.Length rodLength = 0.175 "Length of rod" annotation(Dialog(group = "Rod"));
  parameter SI.Length rodCenterOfMass = rodLength / 2 "Distance from frame_a to center of mass of piston" annotation(Dialog(group = "Rod"));
  parameter SI.Mass rodMass(min = 0) = 1 "Mass of rod" annotation(Dialog(group = "Rod"));
  parameter SI.Inertia rodInertia_11(min = 0) = 0.006 "Inertia 11 of rod with respect to center of mass frame, parallel to frame_a" annotation(Dialog(group = "Rod"));
  parameter SI.Inertia rodInertia_22(min = 0) = 0.0005 "Inertia 22 of rod with respect to center of mass frame, parallel to frame_a" annotation(Dialog(group = "Rod"));
  parameter SI.Inertia rodInertia_33(min = 0) = 0.006 "Inertia 33 of rod with respect to center of mass frame, parallel to frame_a" annotation(Dialog(group = "Rod"));
  final parameter SI.Length cylinderLength = cylinderTopPosition - (pistonLength + rodLength - crankPinOffset) "Maximum length of cylinder volume";
  UWmBody.UWParts.FixedTranslation Mid(animation = false, r = {crankLength - crankPinLength / 2, crankPinOffset, 0}) annotation(Placement(transformation(extent = {{-44, -30}, {-24, -10}})));
  UWmBody.UWParts.FixedTranslation Mounting(r = {crankLength, 0, 0}, animation = false) annotation(Placement(transformation(extent = {{-3, 90}, {17, 110}})));
  UWmBody.UWParts.FixedRotation CylinderInclination(r = {crankLength - crankPinLength / 2, 0, 0}, animation = false, rotationType = UWmBody.UWTypes.RotationTypes.RotationAxis, n = {1, 0, 0}, angle = cylinderInclination) annotation(Placement(transformation(extent = {{-44, 30}, {-24, 50}})));
  UWmBody.UWParts.FixedRotation CrankAngle(animation = false, rotationType = UWmBody.UWTypes.RotationTypes.RotationAxis, n = {1, 0, 0}, angle = crankAngleOffset) annotation(Placement(transformation(extent = {{-84, -80}, {-64, -60}})));
  UWJoints.Assemblies.JointRRP jointRRP(n_a = {1, 0, 0}, n_b = {0, -1, 0}, rRod1_ia = {0, rodLength, 0}, animation = false, rRod2_ib = -{0, pistonLength, 0}, s_offset = -cylinderTopPosition) annotation(Placement(transformation(origin = {0, 12}, extent = {{-20, 20}, {20, -20}}, rotation = 90)));
  UWmBody.UWParts.BodyShape Rod(animation = animation, r = {0, rodLength, 0}, r_CM = {0, rodLength / 2, 0}, shapeType = "modelica://Modelica/Resources/Data/Shapes/Engine/rod.dxf", lengthDirection = {1, 0, 0}, widthDirection = {0, 0, -1}, length = rodLength / 1.75, width = rodLength / 1.75, height = rodLength / 1.75, color = {155, 155, 155}, extra = 1, r_shape = {0, 0, 0}, animateSphere = false, m = rodMass, I_11 = rodInertia_11, I_22 = rodInertia_22, I_33 = rodInertia_33) annotation(Placement(transformation(origin = {49, 9}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  UWmBody.UWParts.BodyShape Piston(animation = animation, r = {0, pistonLength, 0}, r_CM = {0, pistonLength / 2, 0}, shapeType = "modelica://Modelica/Resources/Data/Shapes/Engine/piston.dxf", length = 0.08, width = 0.08, height = 0.08, extra = 1, lengthDirection = {1, 0, 0}, widthDirection = {0, 0, -1}, color = {180, 180, 180}, animateSphere = false, m = pistonMass, I_11 = pistonInertia_11, I_22 = pistonInertia_22, I_33 = pistonInertia_33) annotation(Placement(transformation(origin = {50, 50}, extent = {{10, -10}, {-10, 10}}, rotation = 270)));
  GasForce gasForce(L = cylinderLength, d = 0.1) annotation(Placement(transformation(origin = {-1, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  UWmBody.UWParts.FixedTranslation Crank(animation = false, r = {crankLength, 0, 0}) annotation(Placement(transformation(extent = {{-10, -110}, {10, -90}})));
  UWInterfaces.Frame_a cylinder_a annotation(Placement(transformation(extent = {{-116, 84}, {-84, 116}})));
  UWInterfaces.Frame_a cylinder_b annotation(Placement(transformation(extent = {{84, 84}, {116, 116}})));
  UWInterfaces.Frame_a crank_a annotation(Placement(transformation(extent = {{-116, -116}, {-84, -84}})));
  UWInterfaces.Frame_a crank_b annotation(Placement(transformation(extent = {{84, -116}, {116, -84}})));
equation
  connect(jointRRP.frame_ia, Rod.frame_a) annotation(Line(points = {{20, -4}, {49, -4}, {49, -1}}, color = {95, 95, 95}, thickness = 0.5));
  connect(Mid.frame_b, jointRRP.frame_a) annotation(Line(points = {{-24, -20}, {0, -20}, {0, -8}}, color = {95, 95, 95}, thickness = 0.5));
  connect(gasForce.flange_a, jointRRP.axis) annotation(Line(points = {{9, 70}, {16, 70}, {16, 32}}, color = {0, 191, 0}));
  connect(jointRRP.bearing, gasForce.flange_b) annotation(Line(points = {{8, 32}, {8, 52}, {-20, 52}, {-20, 70}, {-11, 70}}, color = {0, 191, 0}));
  connect(jointRRP.frame_ib, Piston.frame_b) annotation(Line(points = {{20, 28}, {30, 28}, {30, 70}, {50, 70}, {50, 60}}, color = {95, 95, 95}, thickness = 0.5));
  connect(jointRRP.frame_b, CylinderInclination.frame_b) annotation(Line(points = {{0, 32}, {1, 32}, {1, 40}, {-24, 40}}, color = {95, 95, 95}, thickness = 0.5));
  connect(CrankAngle.frame_b, Mid.frame_a) annotation(Line(points = {{-64, -70}, {-56, -70}, {-56, -20}, {-44, -20}}, color = {95, 95, 95}, thickness = 0.5));
  connect(cylinder_a, CylinderInclination.frame_a) annotation(Line(points = {{-100, 100}, {-70, 100}, {-70, 40}, {-44, 40}}, color = {95, 95, 95}, thickness = 0.5));
  connect(cylinder_a, Mounting.frame_a) annotation(Line(points = {{-100, 100}, {-3, 100}}, color = {95, 95, 95}, thickness = 0.5));
  connect(cylinder_b, Mounting.frame_b) annotation(Line(points = {{100, 100}, {17, 100}}, color = {95, 95, 95}, thickness = 0.5));
  connect(CrankAngle.frame_a, crank_a) annotation(Line(points = {{-84, -70}, {-89.5, -70}, {-89.5, -100}, {-100, -100}}, color = {95, 95, 95}, thickness = 0.5));
  connect(crank_a, Crank.frame_a) annotation(Line(points = {{-100, -100}, {-10, -100}}, color = {95, 95, 95}, thickness = 0.5));
  connect(Crank.frame_b, crank_b) annotation(Line(points = {{10, -100}, {100, -100}}, color = {95, 95, 95}, thickness = 0.5));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{-60, -61}, {-60, 64}, {60, 64}, {60, -61}, {100, -61}, {100, 114}, {-100, 114}, {-100, -61}, {-60, -61}}), Ellipse(visible = true, lineColor = {192, 192, 192}, extent = {{-41, -139}, {39, -59}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-156, 116}, {158, 156}}, textString = "%name"), Line(visible = true, points = {{-100, -100}, {100, -100}}, thickness = 0.5), Polygon(visible = true, origin = {0, 2}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.VerticalCylinder, points = {{-58, 58}, {58, 58}, {58, -22}, {50, -22}, {40, -14}, {-40, -14}, {-50, -22}, {-58, -22}}), Rectangle(visible = true, origin = {0, -30}, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, extent = {{-60, 75}, {60, 81}}), Rectangle(visible = true, origin = {0, -30}, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, extent = {{-60, 61}, {60, 67}}), Rectangle(visible = true, origin = {0, -30}, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, extent = {{-60, 49}, {60, 55}}), Ellipse(visible = true, origin = {0, 4}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, extent = {{-10, -10}, {10, 10}}), Line(visible = true, points = {{0, -100}, {25, -67}, {0, 4}})}), Documentation(info = "<html>
<p>
Slider-crank mechanism with analytic handling of kinematic loop to model one cylinder in an engine.
</p>
</html>"));
end CylinderBase;
