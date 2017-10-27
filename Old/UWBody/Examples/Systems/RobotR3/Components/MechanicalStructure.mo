within UWBody.Examples.Systems.RobotR3.Components;

model MechanicalStructure "Model of the mechanical part of the r3 robot (without animation)"
  import Modelica.SIunits.Conversions.to_unit1;
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter SI.Mass mLoad(min = 0) = 15 "Mass of load";
  parameter SI.Position rLoad[3] = {0, 0.25, 0} "Distance from last flange to load mass>";
  parameter Modelica.SIunits.Acceleration g = 9.81 "Gravity acceleration";
  Modelica.SIunits.Angle q[6] "Joint angles";
  SI.AngularVelocity qd[6] "Joint speeds";
  SI.AngularAcceleration qdd[6] "Joint accelerations";
  SI.Torque tau[6] "Joint driving torques";
  //r0={0,0.351,0},
  Modelica.Mechanics.Rotational.Interfaces.Flange_a axis1 annotation(Placement(transformation(extent = {{-220, -180}, {-200, -160}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a axis2 annotation(Placement(transformation(extent = {{-220, -120}, {-200, -100}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a axis3 annotation(Placement(transformation(extent = {{-220, -60}, {-200, -40}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a axis4 annotation(Placement(transformation(extent = {{-220, 0}, {-200, 20}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a axis5 annotation(Placement(transformation(extent = {{-220, 60}, {-200, 80}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a axis6 annotation(Placement(transformation(extent = {{-220, 120}, {-200, 140}})));
  inner UWBody.World world(g = g * Modelica.Math.Vectors.length({0, -1, 0}), n = {0, -1, 0}, animateWorld = false, animateGravity = false, enableAnimation = animation) annotation(Placement(transformation(extent = {{-100, -200}, {-80, -180}})));
  UWBody.Joints.Revolute r1(n = {0, 1, 0}, useAxisFlange = true, animation = animation) annotation(Placement(transformation(origin = {-70, -160}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  UWBody.Joints.Revolute r2(n = {1, 0, 0}, useAxisFlange = true, animation = animation) annotation(Placement(transformation(extent = {{-50, -110}, {-30, -90}})));
  UWBody.Joints.Revolute r3(n = {1, 0, 0}, useAxisFlange = true, animation = animation) annotation(Placement(transformation(origin = {-50, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  UWBody.Joints.Revolute r4(n = {0, 1, 0}, useAxisFlange = true, animation = animation) annotation(Placement(transformation(origin = {-70, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  UWBody.Joints.Revolute r5(n = {1, 0, 0}, useAxisFlange = true, animation = animation) annotation(Placement(transformation(extent = {{-60, 70}, {-40, 90}})));
  UWBody.Joints.Revolute r6(n = {0, 1, 0}, useAxisFlange = true, animation = animation) annotation(Placement(transformation(origin = {-60, 130}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  UWBody.Parts.BodyShape b0(r = {0, 0.351, 0}, shapeType = "modelica://Modelica/Resources/Data/Shapes/RobotR3/b0.dxf", r_shape = {0, 0, 0}, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, length = 0.225, width = 0.3, height = 0.3, color = {0, 0, 255}, animation = animation, animateSphere = false, r_CM = {0, 0, 0}, m = 1) annotation(Placement(transformation(origin = {-30, -170}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  UWBody.Parts.BodyShape b1(r = {0, 0.324, 0.3}, I_22 = 1.16, shapeType = "modelica://Modelica/Resources/Data/Shapes/RobotR3/b1.dxf", lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, length = 0.25, width = 0.15, height = 0.2, animation = animation, animateSphere = false, color = {255, 0, 0}, r_CM = {0, 0, 0}, m = 1) annotation(Placement(transformation(origin = {-70, -118}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  UWBody.Parts.BodyShape b2(r = {0, 0.65, 0}, r_CM = {0.172, 0.205, 0}, m = 56.5, I_11 = 2.58, I_22 = 0.64, I_33 = 2.73, I_21 = -0.46, shapeType = "modelica://Modelica/Resources/Data/Shapes/RobotR3/b2.dxf", r_shape = {0, 0, 0}, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, length = 0.5, width = 0.2, height = 0.15, animation = animation, animateSphere = false, color = {255, 178, 0}) annotation(Placement(transformation(origin = {-16, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  UWBody.Parts.BodyShape b3(r = {0, 0.414, -0.155}, r_CM = {0.064, -0.034, 0}, m = 26.4, I_11 = 0.279, I_22 = 0.245, I_33 = 0.413, I_21 = -0.070, shapeType = "modelica://Modelica/Resources/Data/Shapes/RobotR3/b3.dxf", r_shape = {0, 0, 0}, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, length = 0.15, width = 0.15, height = 0.15, animation = animation, animateSphere = false, color = {255, 0, 0}) annotation(Placement(transformation(origin = {-86, -22}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
  UWBody.Parts.BodyShape b4(r = {0, 0.186, 0}, r_CM = {0, 0, 0}, m = 28.7, I_11 = 1.67, I_22 = 0.081, I_33 = 1.67, shapeType = "modelica://Modelica/Resources/Data/Shapes/RobotR3/b4.dxf", r_shape = {0, 0, 0}, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, length = 0.73, width = 0.1, height = 0.1, animation = animation, animateSphere = false, color = {255, 178, 0}) annotation(Placement(transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  UWBody.Parts.BodyShape b5(r = {0, 0.125, 0}, r_CM = {0, 0, 0}, m = 5.2, I_11 = 1.25, I_22 = 0.81, I_33 = 1.53, shapeType = "modelica://Modelica/Resources/Data/Shapes/RobotR3/b5.dxf", r_shape = {0, 0, 0}, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, length = 0.225, width = 0.075, height = 0.1, animation = animation, animateSphere = false, color = {0, 0, 255}) annotation(Placement(transformation(origin = {-20, 98}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  UWBody.Parts.BodyShape b6(r = {0, 0, 0}, r_CM = {0.05, 0.05, 0.05}, m = 0.5, shapeType = "modelica://Modelica/Resources/Data/Shapes/RobotR3/b6.dxf", r_shape = {0, 0, 0}, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, animation = animation, animateSphere = false, color = {0, 0, 255}) annotation(Placement(transformation(origin = {-60, 160}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  UWBody.Parts.BodyShape load(r = {0, 0, 0}, r_CM = rLoad, m = mLoad, r_shape = {0, 0, 0}, widthDirection = {1, 0, 0}, width = 0.05, height = 0.05, color = {255, 0, 0}, lengthDirection = to_unit1(rLoad), length = Modelica.Math.Vectors.length(rLoad), animation = animation) annotation(Placement(transformation(origin = {-60, 188}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
equation
  connect(r6.frame_b, b6.frame_a) annotation(Line(points = {{-60, 140}, {-60, 150}}, color = {95, 95, 95}, thickness = 0.5));
  q = {r1.phi, r2.phi, r3.phi, r4.phi, r5.phi, r6.phi};
  qd = der(q);
  qdd = der(qd);
  tau = {r1.tau, r2.tau, r3.tau, r4.tau, r5.tau, r6.tau};
  connect(load.frame_a, b6.frame_b) annotation(Line(points = {{-60, 178}, {-60, 170}}, color = {95, 95, 95}, thickness = 0.5));
  connect(world.frame_b, b0.frame_a) annotation(Line(points = {{-80, -190}, {-30, -190}, {-30, -180}}, color = {95, 95, 95}, thickness = 0.5));
  connect(b0.frame_b, r1.frame_a) annotation(Line(points = {{-30, -160}, {-30, -146}, {-48, -146}, {-48, -180}, {-70, -180}, {-70, -170}}, color = {95, 95, 95}, thickness = 0.5));
  connect(b1.frame_b, r2.frame_a) annotation(Line(points = {{-70, -108}, {-70, -100}, {-50, -100}}, color = {95, 95, 95}, thickness = 0.5));
  connect(r1.frame_b, b1.frame_a) annotation(Line(points = {{-70, -150}, {-70, -128}}, color = {95, 95, 95}, thickness = 0.5));
  connect(r2.frame_b, b2.frame_a) annotation(Line(points = {{-30, -100}, {-16, -100}, {-16, -80}}, color = {95, 95, 95}, thickness = 0.5));
  connect(b2.frame_b, r3.frame_a) annotation(Line(points = {{-16, -60}, {-16, -36}, {-40, -36}}, color = {95, 95, 95}, thickness = 0.5));
  connect(r2.axis, axis2) annotation(Line(points = {{-40, -90}, {-42, -90}, {-42, -80}, {-160, -80}, {-160, -110}, {-210, -110}}));
  connect(r1.axis, axis1) annotation(Line(points = {{-80, -160}, {-160, -160}, {-160, -170}, {-210, -170}}));
  connect(r3.frame_b, b3.frame_a) annotation(Line(points = {{-60, -36}, {-88, -36}, {-86, -32}}, color = {95, 95, 95}, thickness = 0.5));
  connect(b3.frame_b, r4.frame_a) annotation(Line(points = {{-86, -12}, {-86, -8}, {-70, -8}, {-70, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(r3.axis, axis3) annotation(Line(points = {{-50, -46}, {-50, -50}, {-210, -50}}));
  connect(r4.axis, axis4) annotation(Line(points = {{-80, 10}, {-210, 10}}));
  connect(r4.frame_b, b4.frame_a) annotation(Line(points = {{-70, 20}, {-70, 40}}, color = {95, 95, 95}, thickness = 0.5));
  connect(b4.frame_b, r5.frame_a) annotation(Line(points = {{-70, 60}, {-70, 80}, {-60, 80}}, color = {95, 95, 95}, thickness = 0.5));
  connect(r5.axis, axis5) annotation(Line(points = {{-50, 90}, {-50, 94}, {-160, 94}, {-160, 70}, {-210, 70}}));
  connect(r5.frame_b, b5.frame_a) annotation(Line(points = {{-40, 80}, {-20, 80}, {-20, 88}}, color = {95, 95, 95}, thickness = 0.5));
  connect(b5.frame_b, r6.frame_a) annotation(Line(points = {{-20, 108}, {-20, 116}, {-60, 116}, {-60, 120}}, color = {95, 95, 95}, thickness = 0.5));
  connect(r6.axis, axis6) annotation(Line(points = {{-70, 130}, {-210, 130}}));
  annotation(Documentation(info = "<html>
<p>
This model contains the mechanical components of the r3 robot
(multibody system).
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-200, -200}, {200, 200}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-200, -200}, {200, 200}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-200, 210}, {200, 250}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-200, -190}, {-140, -150}}, textString = "1"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-200, -70}, {-140, -30}}, textString = "3"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-200, -130}, {-140, -90}}, textString = "2"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-200, 50}, {-140, 90}}, textString = "5"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-200, -12}, {-140, 28}}, textString = "4"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-198, 110}, {-138, 150}}, textString = "6"), Bitmap(visible = true, fileName = "modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Systems/robot_kr15.png", imageSource = "", extent = {{-130, -195}, {195, 195}})}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-200, -200}, {200, 200}})));
end MechanicalStructure;
