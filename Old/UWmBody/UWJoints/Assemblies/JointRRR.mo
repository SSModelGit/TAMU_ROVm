within UWmBody.UWJoints.Assemblies;

model JointRRR "Planar revolute - revolute - revolute joint aggregation (no constraints, no potential states)"
  import UWmBody.UWTypes;
  import Modelica.SIunits.Conversions.to_unit1;
  extends UWInterfaces.PartialTwoFramesDoubleSize;
  UWmBody.UWInterfaces.Frame_a frame_ia "Coordinate system at origin of frame_a fixed at connecting rod of left and middle revolute joint" annotation(Placement(transformation(origin = {-80, 100}, extent = {{-8, -8}, {8, 8}}, rotation = 90), visible = true, iconTransformation(origin = {-80, 100}, extent = {{-8, -8}, {8, 8}}, rotation = 90)));
  UWmBody.UWInterfaces.Frame_b frame_ib "Coordinate system at origin of frame_b fixed at connecting rod of middle and right revolute joint" annotation(Placement(transformation(origin = {80, 100}, extent = {{-8, 8}, {8, -8}}, rotation = 270), visible = true, iconTransformation(origin = {80, 100}, extent = {{-8, 8}, {8, -8}}, rotation = 270)));
  UWmBody.UWInterfaces.Frame_b frame_im "Coordinate system at origin of revolute joint in the middle fixed at connecting rod of middle and right revolute joint" annotation(Placement(transformation(origin = {0, 100}, extent = {{8, -8}, {-8, 8}}, rotation = 270), visible = true, iconTransformation(origin = {0, 100}, extent = {{8, -8}, {-8, 8}}, rotation = 270)));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a axis "1-dim. rotational flange that drives the right revolute joint at frame_b" annotation(Placement(transformation(extent = {{105, 85}, {95, 75}}, origin = {0, 0}, rotation = 0), visible = true, iconTransformation(origin = {0, 0}, extent = {{105, 85}, {95, 75}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b bearing "1-dim. rotational flange of the drive bearing of the right revolute joint at frame_b" annotation(Placement(transformation(extent = {{95, 45}, {105, 35}}, origin = {0, 0}, rotation = 0), visible = true, iconTransformation(origin = {0, 0}, extent = {{95, 45}, {105, 35}}, rotation = 0)));
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter UWmBody.UWTypes.Axis n_a = {0, 0, 1} "Axes of revolute joints resolved in frame_a (all axes are parallel to each other)" annotation(Evaluate = true);
  final parameter Real n_b[3](each final unit = "1", each fixed = false, start = {0, 0, 1}) "Axis of revolute joint fixed and resolved in frame_b" annotation(Evaluate = true);
  parameter SI.Position rRod1_ia[3] = {1, 0, 0} "Vector from origin of frame_a to revolute joint in the middle, resolved in frame_ia" annotation(Evaluate = true);
  parameter SI.Position rRod2_ib[3] = {-1, 0, 0} "Vector from origin of frame_ib to revolute joint in the middle, resolved in frame_ib";
  parameter Cv.NonSIunits.Angle_deg phi_offset = 0 "Relative angle offset of revolute joint at frame_b (angle = phi(t) + from_deg(phi_offset))";
  parameter Cv.NonSIunits.Angle_deg phi_guess = 0 "Select the configuration such that at initial time |phi(t0) - from_deg(phi_guess)| is minimal";
  parameter SI.Distance cylinderLength = world.defaultJointLength "Length of cylinders representing the revolute joints" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Distance cylinderDiameter = world.defaultJointWidth "Diameter of cylinders representing the revolute joints" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.Color cylinderColor = UWmBody.UWTypes.Defaults.JointColor "Color of cylinders representing the revolute joints" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Diameter rodDiameter = 1.1 * cylinderDiameter "Diameter of the two rods connecting the revolute joints" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.Color rodColor = UWmBody.UWTypes.Defaults.RodColor "Color of the two rods connecting the revolute joint" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter Boolean checkTotalPower = false "= true, if total power flowing into this component shall be determined (must be zero)" annotation(Dialog(tab = "Advanced"));
  final parameter Real e_a[3](each final unit = "1") = Modelica.Math.Vectors.normalizeWithAssert(n_a) "Unit vector along axes of rotations, resolved in frame_a";
  final parameter Real e_ia[3](each final unit = "1") = jointUSR.e2_ia "Unit vector along axes of rotations, resolved in frame_ia";
  final parameter Real e_b[3](each final unit = "1") = jointUSR.revolute.e "Unit vector along axes of rotations, resolved in frame_b, frame_ib and frame_im";
  SI.Power totalPower = jointUSR.totalPower "Total power flowing into this element, if checkTotalPower=true (otherwise dummy)";
  JointUSR jointUSR(animation = false, n1_a = n_a, n_b = n_b, phi_offset = phi_offset, rRod2_ib = rRod2_ib, showUniversalAxes = false, rRod1_ia = rRod1_ia, checkTotalPower = checkTotalPower, phi_guess = phi_guess) annotation(Placement(transformation(extent = {{-30, -20}, {10, 20}})));
protected
  UWVisualizers.Advanced.Shape shape_rev1(shapeType = "cylinder", color = cylinderColor, specularCoefficient = specularCoefficient, length = cylinderLength, width = cylinderDiameter, height = cylinderDiameter, lengthDirection = e_a, widthDirection = {0, 1, 0}, r_shape = -e_a * (cylinderLength / 2), r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
  UWVisualizers.Advanced.Shape shape_rev2(shapeType = "cylinder", color = cylinderColor, specularCoefficient = specularCoefficient, length = cylinderLength, width = cylinderDiameter, height = cylinderDiameter, lengthDirection = e_b, widthDirection = {0, 1, 0}, r_shape = -e_b * (cylinderLength / 2), r = frame_im.r_0, R = frame_im.R) if world.enableAnimation and animation;
  UWVisualizers.Advanced.Shape shape_rev3(shapeType = "cylinder", color = cylinderColor, specularCoefficient = specularCoefficient, length = cylinderLength, width = cylinderDiameter, height = cylinderDiameter, lengthDirection = e_b, widthDirection = {0, 1, 0}, r_shape = -e_b * (cylinderLength / 2), r = frame_b.r_0, R = frame_b.R) if world.enableAnimation and animation;
  UWVisualizers.Advanced.Shape shape_rod1(shapeType = "cylinder", color = rodColor, specularCoefficient = specularCoefficient, length = Modelica.Math.Vectors.length(rRod1_ia), width = rodDiameter, height = rodDiameter, lengthDirection = to_unit1(rRod1_ia), widthDirection = e_ia, r = frame_ia.r_0, R = frame_ia.R) if world.enableAnimation and animation;
  UWVisualizers.Advanced.Shape shape_rod2(shapeType = "cylinder", color = rodColor, specularCoefficient = specularCoefficient, length = Modelica.Math.Vectors.length(rRod2_ib), width = rodDiameter, height = rodDiameter, lengthDirection = to_unit1(rRod2_ib), widthDirection = e_b, r = frame_ib.r_0, R = frame_ib.R) if world.enableAnimation and animation;
initial equation
  n_b = Frames.resolve2(frame_b.R, Frames.resolve1(frame_a.R, n_a));
equation
  connect(jointUSR.frame_a, frame_a) annotation(Line(points = {{-30, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(jointUSR.frame_b, frame_b) annotation(Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(jointUSR.frame_ia, frame_ia) annotation(Line(points = {{-26, 20}, {-26, 70}, {-80, 70}, {-80, 100}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(jointUSR.frame_im, frame_im) annotation(Line(points = {{-10, 20}, {-10, 70}, {0, 70}, {0, 100}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(jointUSR.frame_ib, frame_ib) annotation(Line(points = {{6, 20}, {6, 50}, {80, 50}, {80, 100}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(jointUSR.axis, axis) annotation(Line(points = {{10, 16}, {86, 16}, {86, 80}, {100, 80}}, visible = true));
  connect(jointUSR.bearing, bearing) annotation(Line(points = {{10, 8}, {94, 8}, {94, 40}, {100, 40}}, visible = true));
  annotation(Documentation(info = "<html>
<p>
This component consists of <b>3 revolute</b> joints with parallel
axes of rotation that are connected together by two rods, see the default
animation in the following figure (the axes vectors are not part of the
default animation):
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/JointRRR.png\" ALT=\"model Joints.Assemblies.JointRRR\">
</p>

<p>
This joint aggregation introduces neither constraints nor state variables and
should therefore be used in kinematic loops whenever possible to
avoid non-linear systems of equations. It is only meaningful to
use this component in <b>planar loops</b>. Basically, the position
and orientation of the 3 revolute joints as well as of frame_ia, frame_ib, and
frame_im are calculated by solving analytically a non-linear equation,
given the position and orientation at frame_a and at frame_b.
</p>
<p>
Connector <b>frame_a</b> is the \"left\" side of the first revolute joint
whereas <b>frame_ia</b> is the \"right side of this revolute joint, fixed in rod 1.
Connector <b>frame_b</b> is the \"right\" side of the third revolute joint
whereas <b>frame_ib</b> is the \"left\" side of this revolute joint, fixed in rod 2.
Finally, connector <b>frame_im</b> is the connector at the \"right\" side
of the revolute joint in the middle, fixed in rod 2.
</p>
<p>
The easiest way to define the parameters of this joint is by moving the
MultiBody system in a <b>reference configuration</b> where <b>all frames</b>
of all components are <b>parallel</b> to each other (alternatively,
at least frame_a, frame_ia, frame_im, frame_ib, frame_b of the JointRRR joint
should be parallel to each other when defining an instance of this
component).
</p>
<p>
Basically, the JointRRR model consists internally of a universal -
spherical - revolute joint aggregation (= JointUSR). In a planar
loop this will behave as if 3 revolute joints with parallel axes
are connected by rigid rods.
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.2, grid = {5, 5}), graphics = {Rectangle(visible = true, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-90, -90}, {90, 90}}), Line(visible = true, points = {{80, 24}, {80, 80}, {80, 80}, {80, 100}}, color = {64, 64, 64}, thickness = 0.5), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, extent = {{-100, -25}, {-50, 25}}), Rectangle(visible = true, origin = {-75, 0}, rotation = 36, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{0, -5}, {95, 5}}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, extent = {{-25, 30}, {25, 80}}), Rectangle(visible = true, origin = {74.662, -0.955}, rotation = 143, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{0, -5}, {95, 5}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-140, -90}, {140, -50}}, textString = "%name"), Text(visible = true, origin = {4, 0}, textColor = {64, 64, 64}, extent = {{36, 90}, {71, 110}}, textString = "ib"), Text(visible = true, origin = {4, 0}, textColor = {64, 64, 64}, extent = {{-126, 90}, {-87, 110}}, textString = "ia"), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{-85, -10}, {-65, 10}}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, extent = {{50, -25}, {100, 25}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-128, -47}, {136, -29}}, textString = "n_a=%n_a"), Line(visible = true, points = {{0, 57}, {0, 86}, {0, 86}, {0, 100}}, color = {64, 64, 64}, thickness = 0.5), Text(visible = true, origin = {4, 0}, textColor = {64, 64, 64}, extent = {{-46, 90}, {-7, 110}}, textString = "im"), Line(visible = true, points = {{-80, 100}, {-80, 8}}, color = {64, 64, 64}, thickness = 0.5), Line(visible = true, points = {{80, 80}, {100, 80}}, color = {64, 64, 64}, thickness = 0.5), Line(visible = true, points = {{100, 40}, {93, 40}, {93, 3}}, color = {64, 64, 64}, thickness = 0.5), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{75, -5}, {97.5, 5}}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{65, -10}, {85, 10}}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{-10, 45}, {10, 65}})}));
end JointRRR;
