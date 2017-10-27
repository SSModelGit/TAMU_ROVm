within UWmBody.UWJoints;

model Planar "Planar joint (3 degrees-of-freedom, 6 potential states)"
  extends UWmBody.UWInterfaces.PartialTwoFrames;
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter UWmBody.UWTypes.Axis n = {0, 0, 1} "Axis orthogonal to unconstrained plane, resolved in frame_a (= same as in frame_b)" annotation(Evaluate = true);
  parameter UWmBody.UWTypes.Axis n_x = {1, 0, 0} "Vector in direction of x-axis of plane, resolved in frame_a (n_x shall be orthogonal to n)" annotation(Evaluate = true);
  parameter SI.Distance cylinderLength = world.defaultJointLength "Length of revolute cylinder" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Distance cylinderDiameter = world.defaultJointWidth "Diameter of revolute cylinder" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.Color cylinderColor = UWmBody.UWTypes.Defaults.JointColor "Color of revolute cylinder" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Distance boxWidth = 0.3 * cylinderDiameter "Width of prismatic joint boxes" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Distance boxHeight = boxWidth "Height of prismatic joint boxes" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.Color boxColor = UWmBody.UWTypes.Defaults.JointColor "Color of prismatic joint boxes" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  parameter StateSelect stateSelect = StateSelect.prefer "Priority to use joint coordinates (s_x, s_y, phi, v_x, v_y, w) as states" annotation(Dialog(tab = "Advanced"));
  Prismatic prismatic_x(stateSelect = StateSelect.never, n = cross(cross(n, n_x), n), animation = false) annotation(Placement(transformation(extent = {{-69, -20}, {-29, 20}})));
  Prismatic prismatic_y(stateSelect = StateSelect.never, n = cross(n, n_x), animation = false) annotation(Placement(transformation(origin = {0, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  Revolute revolute(stateSelect = StateSelect.never, n = n, animation = false) annotation(Placement(transformation(extent = {{41, -20}, {81, 20}})));
  SI.Position s_x(start = 0, stateSelect = stateSelect) "Relative distance along first prismatic joint starting at frame_a";
  SI.Position s_y(start = 0, stateSelect = stateSelect) "Relative distance along second prismatic joint starting at first prismatic joint";
  Modelica.SIunits.Angle phi(start = 0, stateSelect = stateSelect) "Relative rotation angle from frame_a to frame_b";
  Modelica.SIunits.Velocity v_x(start = 0, stateSelect = stateSelect) "First derivative of s_x (relative velocity in s_x direction)";
  Modelica.SIunits.Velocity v_y(start = 0, stateSelect = stateSelect) "First derivative of s_y (relative velocity in s_y direction)";
  SI.AngularVelocity w(start = 0, stateSelect = stateSelect) "First derivative of angle phi (relative angular velocity)";
  Modelica.SIunits.Acceleration a_x(start = 0) "Second derivative of s_x (relative acceleration in s_x direction)";
  Modelica.SIunits.Acceleration a_y(start = 0) "Second derivative of s_y (relative acceleration in s_y direction)";
  SI.AngularAcceleration wd(start = 0) "Second derivative of angle phi (relative angular acceleration)";
protected
  parameter Integer ndim = if world.enableAnimation and animation then 1 else 0;
  parameter Real e[3](each final unit = "1") = Modelica.Math.Vectors.normalize(n);
protected
  UWVisualizers.Advanced.Shape box_x[ndim](each shapeType = "box", each color = boxColor, each length = prismatic_x.s, each width = boxWidth, each height = boxWidth, each lengthDirection = prismatic_x.e, each widthDirection = {0, 1, 0}, each r = frame_a.r_0, each R = frame_a.R) annotation(Placement(transformation(extent = {{-80, 30}, {-60, 50}})));
  UWVisualizers.Advanced.Shape box_y[ndim](each shapeType = "box", each color = boxColor, each length = prismatic_y.s, each width = boxWidth, each height = boxWidth, each lengthDirection = prismatic_y.e, each widthDirection = {1, 0, 0}, each r = prismatic_y.frame_a.r_0, each R = prismatic_y.frame_a.R) annotation(Placement(transformation(extent = {{-46, 69}, {-26, 89}})));
  UWVisualizers.Advanced.Shape cylinder[ndim](each shapeType = "cylinder", each color = cylinderColor, each length = cylinderLength, each width = cylinderDiameter, each height = cylinderDiameter, each lengthDirection = n, each widthDirection = {0, 1, 0}, each r_shape = -e * (cylinderLength / 2), each r = revolute.frame_b.r_0, each R = revolute.frame_b.R) annotation(Placement(transformation(extent = {{50, 30}, {70, 50}})));
equation
  s_x = prismatic_x.s;
  s_y = prismatic_y.s;
  phi = revolute.phi;
  v_x = der(s_x);
  v_y = der(s_y);
  w = der(phi);
  a_x = der(v_x);
  a_y = der(v_y);
  wd = der(w);
  connect(frame_a, prismatic_x.frame_a) annotation(Line(points = {{-100, 0}, {-84, 0}, {-84, 0}, {-69, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(prismatic_x.frame_b, prismatic_y.frame_a) annotation(Line(points = {{-29, 0}, {0, 0}, {0, 30}}, color = {95, 95, 95}, thickness = 0.5));
  connect(prismatic_y.frame_b, revolute.frame_a) annotation(Line(points = {{0, 70}, {0, 80}, {30, 80}, {30, 0}, {41, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(revolute.frame_b, frame_b) annotation(Line(points = {{81, 0}, {92, 0}, {92, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  annotation(Documentation(info = "<html>
<p>
Joint where frame_b can move in a plane and can rotate around an
axis orthogonal to the plane. The plane is defined by
vector n which is perpendicular to the plane and by vector n_x,
which points in the direction of the x-axis of the plane.
frame_a and frame_b coincide when s_x=prismatic_x.s=0,
s_y=prismatic_y.s=0 and phi=revolute.phi=0. This joint has the following
potential states:
</p>
<ul>
<li> the relative distance s_x = prismatic_x.s [m] along axis n_x, </li>
<li> the relative distance s_y = prismatic_y.s [m] along axis n_y = cross(n,n_x), </li>
<li> the relative angle phi = revolute.phi [rad] around axis n, </li>
<li> the relative velocity v_x (= der(s_x)).</li>
<li> the relative velocity v_y (= der(s_y)).</li>
<li> the relative angular velocity w (= der(phi))</li>
</ul>
<p>
They are used as candidates for automatic selection of states
from the tool. This may be enforced by setting \"stateSelect=StateSelect.<b>always</b>\"
in the <b>Advanced</b> menu. The states are usually selected automatically.
In certain situations, especially when closed kinematic loops are present,
it might be slightly more efficient, when using the \"StateSelect.always\" setting.
</p>
<p>
In the following figure the animation of a planar
joint is shown. The light blue coordinate system is
frame_a and the dark blue coordinate system is
frame_b of the joint. The black arrows are parameter
vectors \"n\" and \"n_x\"
(here: n = {0,1,0}, n_x = {0,0,1}, s_x.start = 0.5,
s_y.start = 0.5, phi.start = 45<sup>o</sup>).
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/Planar.png\">
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -105}, {150, -75}}, textString = "n=%n"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 70}, {150, 110}}, textString = "%name"), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, extent = {{-15, -60}, {0, 60}}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, extent = {{0, -50}, {15, 50}}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, -15}, {-15, 15}}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{15, -15}, {100, 15}}), Line(visible = true, origin = {50, 0}, points = {{0, -50}, {0, 50}}, color = {64, 64, 64}, arrow = {Arrow.Filled, Arrow.Filled}, arrowSize = 20)}));
end Planar;
