within RBodyInFluid.Parts;

model BasicBodyShape "Rigid body with mass, inertia tensor, different shapes for animation, and two frame connectors (12 potential states)"
  import Modelica.Mechanics.MultiBody.Types;
  import Modelica.SIunits.Conversions.to_unit1;
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a "Coordinate system fixed to the component with one cut-force and cut-torque" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b "Coordinate system fixed to the component with one cut-force and cut-torque" annotation(Placement(transformation(extent = {{84, -16}, {116, 16}})));
  parameter Boolean animation = true "= true, if animation shall be enabled (show shape between frame_a and frame_b and optionally a sphere at the center of mass)";
  parameter Boolean animateSphere = true "= true, if mass shall be animated as sphere provided animation=true";
  parameter SI.Position r[3](start = {0, 0, 0}) "Vector from frame_a to frame_b resolved in frame_a";
  parameter SI.Position r_CM[3](start = {0, 0, 0}) "Vector from frame_a to center of mass, resolved in frame_a";
  parameter SI.Mass m(min = 0, start = 1) "Mass of rigid body";
  parameter SI.Density density(min = 0, start = 1000) = 1000 "Density of rigid body";
  parameter SI.DimensionlessRatio c_d(min = 0, start = 1) = 1 "Viscous drag coefficient of rigid body";
  parameter SI.Area A(min = 0, start = 1) "Cross sectional area of rigid body";
  parameter SI.Inertia I_11(min = 0) = 0.001 "(1,1) element of inertia tensor" annotation(Dialog(group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
  parameter SI.Inertia I_22(min = 0) = 0.001 "(2,2) element of inertia tensor" annotation(Dialog(group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
  parameter SI.Inertia I_33(min = 0) = 0.001 "(3,3) element of inertia tensor" annotation(Dialog(group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
  parameter SI.Inertia I_21(min = -C.inf) = 0 "(2,1) element of inertia tensor" annotation(Dialog(group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
  parameter SI.Inertia I_31(min = -C.inf) = 0 "(3,1) element of inertia tensor" annotation(Dialog(group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
  parameter SI.Inertia I_32(min = -C.inf) = 0 "(3,2) element of inertia tensor" annotation(Dialog(group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
  SI.Position r_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Position vector from origin of world frame to origin of frame_a" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Absolute velocity of frame_a, resolved in world frame (= der(r_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_a resolved in world frame (= der(v_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  parameter Boolean angles_fixed = false "= true, if angles_start are used as initial values, else as guess values" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
  parameter SI.Angle angles_start[3] = {0, 0, 0} "Initial values of angles to rotate frame_a around 'sequence_start' axes into frame_b" annotation(Dialog(tab = "Initialization"));
  parameter Types.RotationSequence sequence_start = {1, 2, 3} "Sequence of rotations to rotate frame_a into frame_b at initial time" annotation(Evaluate = true, Dialog(tab = "Initialization"));
  parameter Boolean w_0_fixed = false "= true, if w_0_start are used as initial values, else as guess values" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
  parameter SI.AngularVelocity w_0_start[3] = {0, 0, 0} "Initial or guess values of angular velocity of frame_a resolved in world frame" annotation(Dialog(tab = "Initialization"));
  parameter Boolean z_0_fixed = false "= true, if z_0_start are used as initial values, else as guess values" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
  parameter SI.AngularAcceleration z_0_start[3] = {0, 0, 0} "Initial values of angular acceleration z_0 = der(w_0)" annotation(Dialog(tab = "Initialization"));
  parameter Types.ShapeType shapeType = "cylinder" "Type of shape" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Position r_shape[3] = {0, 0, 0} "Vector from frame_a to shape origin, resolved in frame_a" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter Types.Axis lengthDirection = to_unit1(r - r_shape) "Vector in length direction of shape, resolved in frame_a" annotation(Evaluate = true, Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter Types.Axis widthDirection = {0, 1, 0} "Vector in width direction of shape, resolved in frame_a" annotation(Evaluate = true, Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Length length = Modelica.Math.Vectors.length(r - r_shape) "Length of shape" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Distance width = length / world.defaultWidthFraction "Width of shape" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Distance height = width "Height of shape" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter Types.ShapeExtra extra = 0.0 "Additional parameter depending on shapeType (see docu of Visualizers.Advanced.Shape)" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input Types.Color color = Modelica.Mechanics.MultiBody.Types.Defaults.BodyColor "Color of shape" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Diameter sphereDiameter = 2 * width "Diameter of sphere" annotation(Dialog(tab = "Animation", group = "if animation = true and animateSphere = true", enable = animation and animateSphere));
  input Types.Color sphereColor = color "Color of sphere of mass" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true and animateSphere = true", enable = animation and animateSphere));
  input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(Dialog(tab = "Advanced"));
  parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(Dialog(tab = "Advanced"));
  parameter Types.RotationSequence sequence_angleStates = {1, 2, 3} "Sequence of rotations to rotate world frame into frame_a around the 3 angles used as potential states" annotation(Evaluate = true, Dialog(tab = "Advanced", enable = not useQuaternions));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation frameTranslation(r = r, animation = false) annotation(Placement(transformation(extent = {{-20, -20}, {20, 20}})));
  BasicBody body(r_CM = r_CM, m = m, I_11 = I_11, I_22 = I_22, I_33 = I_33, I_21 = I_21, I_31 = I_31, I_32 = I_32, animation = animation, sequence_start = sequence_start, angles_fixed = angles_fixed, angles_start = angles_start, w_0_fixed = w_0_fixed, w_0_start = w_0_start, z_0_fixed = z_0_fixed, z_0_start = z_0_start, useQuaternions = useQuaternions, sequence_angleStates = sequence_angleStates, enforceStates = false, density = density, c_d = c_d, A = A) annotation(Placement(transformation(extent = {{-27.3333, -70.3333}, {13, -30}})));
protected
  outer Modelica.Mechanics.MultiBody.World world;
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape shape1(shapeType = shapeType, color = color, specularCoefficient = specularCoefficient, length = length, width = width, height = height, lengthDirection = lengthDirection, widthDirection = widthDirection, r_shape = r_shape, extra = extra, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape shape2(shapeType = "sphere", color = sphereColor, specularCoefficient = specularCoefficient, length = sphereDiameter, width = sphereDiameter, height = sphereDiameter, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, r_shape = r_CM - {1, 0, 0} * sphereDiameter / 2, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation and animateSphere;
equation
  r_0 = frame_a.r_0;
  v_0 = der(r_0);
  a_0 = der(v_0);
  connect(frame_a, frameTranslation.frame_a) annotation(Line(points = {{-100, 0}, {-20, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(frame_b, frameTranslation.frame_b) annotation(Line(points = {{100, 0}, {20, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(frame_a, body.frame_a) annotation(Line(points = {{-100, 0}, {-60, 0}, {-60, -50.1666}, {-27.3333, -50.1666}}, color = {95, 95, 95}, thickness = 0.5));
  annotation(Documentation(info = "<html>
<p>
<b>Rigid body</b> with mass and inertia tensor and <b>two frame connectors</b>.
All parameter vectors have to be resolved in frame_a.
The <b>inertia tensor</b> has to be defined with respect to a
coordinate system that is parallel to frame_a with the
origin at the center of mass of the body. The coordinate system <b>frame_b</b>
is always parallel to <b>frame_a</b>.
</p>
<p>
By default, this component is visualized by any <b>shape</b> that can be
defined with Modelica.Mechanics.MultiBody.Visualizers.FixedShape. This shape is placed
between frame_a and frame_b (default: length(shape) = Frames.length(r)).
Additionally a <b>sphere</b> may be visualized that has
its center at the center of mass.
Note, that
the animation may be switched off via parameter animation = <b>false</b>.
</p>
<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/BodyShape.png\" ALT=\"Parts.BodyShape\">
</p>

<p>
The following shapes can be defined via parameter <b>shapeType</b>,
e.g., shapeType=\"cone\":
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/FixedShape.png\" ALT=\"Visualizers.FixedShape\">
</p>

<p>
A BodyShape component has potential states. For details of these
states and of the \"Advanced\" menu parameters, see model
<a href=\"modelica://Modelica.Mechanics.MultiBody.Parts.Body\">MultiBody.Parts.Body</a>.
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, fillColor = {0, 0, 100}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Text(visible = true, textColor = {255, 255, 255}, extent = {{-150, 70}, {150, 110}}, textString = "%name"), Text(visible = true, textColor = {255, 255, 255}, extent = {{-150, -100}, {150, -70}}, textString = "d=%d", fontName = "Liberation Sans"), Rectangle(visible = true, lineColor = {10, 90, 224}, fillColor = {156, 203, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, -30}, {101, 31}}, radius = 10), Ellipse(visible = true, lineColor = {10, 90, 224}, fillColor = {156, 203, 255}, fillPattern = FillPattern.Sphere, extent = {{-60, -60}, {60, 60}}), Text(visible = true, textColor = {255, 255, 255}, extent = {{-50, -27}, {55, 24}}, textString = "%m"), Text(visible = true, origin = {10, 0}, textColor = {255, 255, 255}, extent = {{55, -13}, {91, 12}}, textString = "b"), Text(visible = true, origin = {-10, 0}, textColor = {255, 255, 255}, extent = {{-92, -12}, {-56, 13}}, textString = "a"), Line(visible = true, origin = {1, -3}, points = {{-56, 78}, {-56, -72}}, color = {255, 255, 255}, pattern = LinePattern.Dash, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Line(visible = true, origin = {-2, -3}, rotation = -180, points = {{2, 78}, {2, -72}}, color = {255, 255, 255}, pattern = LinePattern.Dash, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Line(visible = true, origin = {9, -65}, rotation = -630, points = {{66, 80}, {66, -70}}, color = {255, 255, 255}, pattern = LinePattern.Dash, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30)}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{-100, 9}, {-100, 43}}, color = {128, 128, 128}), Line(points = {{100, 0}, {100, 44}}, color = {128, 128, 128}), Line(points = {{-100, 40}, {90, 40}}, color = {135, 135, 135}), Polygon(points = {{90, 44}, {90, 36}, {100, 40}, {90, 44}}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid), Text(extent = {{-22, 68}, {20, 40}}, lineColor = {128, 128, 128}, textString = "r"), Line(points = {{-100, -10}, {-100, -90}}, color = {128, 128, 128}), Line(points = {{-100, -84}, {-10, -84}}, color = {128, 128, 128}), Polygon(points = {{-10, -80}, {-10, -88}, {0, -84}, {-10, -80}}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid), Text(extent = {{-82, -66}, {-56, -84}}, lineColor = {128, 128, 128}, textString = "r_CM"), Line(points = {{0, -46}, {0, -90}}, color = {128, 128, 128})}));
end BasicBodyShape;
