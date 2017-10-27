within UWmBody.UWJoints;

model Universal "Universal joint (2 degrees-of-freedom, 4 potential states)"
  extends UWmBody.UWInterfaces.PartialTwoFrames;
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter UWmBody.UWTypes.Axis n_a = {1, 0, 0} "Axis of revolute joint 1 resolved in frame_a" annotation(Evaluate = true);
  parameter UWmBody.UWTypes.Axis n_b = {0, 1, 0} "Axis of revolute joint 2 resolved in frame_b" annotation(Evaluate = true);
  parameter SI.Distance cylinderLength = world.defaultJointLength "Length of cylinders representing the joint axes" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Distance cylinderDiameter = world.defaultJointWidth "Diameter of cylinders representing the joint axes" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.Color cylinderColor = UWmBody.UWTypes.Defaults.JointColor "Color of cylinders representing the joint axes" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter StateSelect stateSelect = StateSelect.prefer "Priority to use joint coordinates (phi_a, phi_b, w_a, w_b) as states" annotation(Dialog(tab = "Advanced"));
  UWmBody.UWJoints.Revolute revolute_a(n = n_a, stateSelect = StateSelect.never, cylinderDiameter = cylinderDiameter, cylinderLength = cylinderLength, cylinderColor = cylinderColor, specularCoefficient = specularCoefficient, animation = animation) annotation(Placement(transformation(extent = {{-60, -25}, {-10, 25}})));
  UWmBody.UWJoints.Revolute revolute_b(n = n_b, stateSelect = StateSelect.never, animation = animation, cylinderDiameter = cylinderDiameter, cylinderLength = cylinderLength, cylinderColor = cylinderColor, specularCoefficient = specularCoefficient) annotation(Placement(transformation(origin = {35, 45}, extent = {{-25, -25}, {25, 25}}, rotation = 90)));
  Modelica.SIunits.Angle phi_a(start = 0, stateSelect = stateSelect) "Relative rotation angle from frame_a to intermediate frame";
  Modelica.SIunits.Angle phi_b(start = 0, stateSelect = stateSelect) "Relative rotation angle from intermediate frame to frame_b";
  SI.AngularVelocity w_a(start = 0, stateSelect = stateSelect) "First derivative of angle phi_a (relative angular velocity a)";
  SI.AngularVelocity w_b(start = 0, stateSelect = stateSelect) "First derivative of angle phi_b (relative angular velocity b)";
  SI.AngularAcceleration a_a(start = 0) "Second derivative of angle phi_a (relative angular acceleration a)";
  SI.AngularAcceleration a_b(start = 0) "Second derivative of angle phi_b (relative angular acceleration b)";
equation
  phi_a = revolute_a.phi;
  phi_b = revolute_b.phi;
  w_a = der(phi_a);
  w_b = der(phi_b);
  a_a = der(w_a);
  a_b = der(w_b);
  connect(frame_a, revolute_a.frame_a) annotation(Line(points = {{-100, 0}, {-60, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(revolute_b.frame_b, frame_b) annotation(Line(points = {{35, 70}, {35, 90}, {70, 90}, {70, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(revolute_a.frame_b, revolute_b.frame_a) annotation(Line(points = {{-10, 0}, {35, 0}, {35, 20}}, color = {95, 95, 95}, thickness = 0.5));
  annotation(Documentation(info = "<html>
<p>
Joint where frame_a rotates around axis n_a which is fixed in frame_a
and frame_b rotates around axis n_b which is fixed in frame_b.
The two frames coincide when
\"revolute_a.phi=0\" and \"revolute_b.phi=0\". This joint
has the following potential states;
</p>
<ul>
<li> The relative angle phi_a = revolute_a.phi [rad] around axis n_a, </li>
<li> the relative angle phi_b = revolute_b.phi [rad] around axis n_b, </li>
<li> the relative angular velocity w_a (= der(phi_a))  and </li>
<li> the relative angular velocity w_b (= der(phi_b)).</li>
</ul>
<p>
They are used as candidates for automatic selection of states
from the tool. This may be enforced by setting \"stateSelect=StateSelect.<b>always</b>\"
in the <b>Advanced</b> menu. The states are usually selected automatically.
In certain situations, especially when closed kinematic loops are present,
it might be slightly more efficient, when using the \"StateSelect.always\" setting.
</p>

<p>
In the following figure the animation of a universal
joint is shown. The light blue coordinate system is
frame_a and the dark blue coordinate system is
frame_b of the joint
(here: n_a = {0,0,1}, n_b = {0,1,0}, phi_a.start = 90<sup>o</sup>,
phi_b.start = 45<sup>o</sup>).
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/Universal.png\">
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -120}, {150, -80}}, textString = "%name"), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-30, 35}, {-35, 30}, {-35, -30}, {-30, -35}, {30, -35}, {35, -30}, {35, 30}, {30, 35}}), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{25, 65}, {-45, 65}, {-65, 45}, {-65, 20}, {-70, 15}, {-100, 15}, {-100, -15}, {-70, -15}, {-65, -20}, {-65, -45}, {-45, -65}, {25, -65}, {25, -45}, {-35, -45}, {-45, -35}, {-45, 35}, {-35, 45}, {25, 45}}), Polygon(visible = true, origin = {51.25, 0}, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{-76.25, 25}, {8.75, 25}, {18.75, 15}, {48.75, 15}, {48.75, -15}, {18.75, -15}, {8.75, -25}, {-76.25, -25}}), Ellipse(visible = true, rotation = 45, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-10, -10}, {10, 10}}), Rectangle(visible = true, origin = {0, -40}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.VerticalCylinder, extent = {{-10, -5}, {10, 5}}), Rectangle(visible = true, origin = {-85, 0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-15, -15}, {15, 15}}), Rectangle(visible = true, origin = {0, 40}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.VerticalCylinder, extent = {{-10, -5}, {10, 5}}), Rectangle(visible = true, origin = {85, 0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-15, -15}, {15, 15}})}));
end Universal;
