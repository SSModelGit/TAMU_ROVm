within UWmBody.UWParts;

model RollingWheel "Ideal rolling wheel on flat surface z=0 (5 positional, 3 velocity degrees of freedom)"
  parameter Boolean animation = true "= true, if animation of wheel shall be enabled";
  parameter SI.Radius wheelRadius "Radius of wheel";
  parameter SI.Mass wheelMass "Mass of wheel";
  parameter SI.Inertia wheel_I_axis "Inertia along the wheel axis";
  parameter SI.Inertia wheel_I_long "Inertia perpendicular to the wheel axis";
  parameter StateSelect stateSelect = StateSelect.always "Priority to use generalized coordinates as states" annotation(HideResult = true, Evaluate = true);
  SI.Position x(start = 0, fixed = true, stateSelect = stateSelect) "x-coordinate of wheel axis";
  SI.Position y(start = 0, fixed = true, stateSelect = stateSelect) "y-coordinate of wheel axis";
  Modelica.SIunits.Angle angles[3](start = {0, 0, 0}, each fixed = true, each stateSelect = stateSelect) "Angles to rotate world-frame in to frame_a around z-, y-, x-axis" annotation(Dialog(group = "Initialization", showStartAttribute = true));
  SI.AngularVelocity der_angles[3](start = {0, 0, 0}, each fixed = true, each stateSelect = stateSelect) "Derivative of angles" annotation(Dialog(group = "Initialization", showStartAttribute = true));
  parameter SI.Distance width = 0.035 "Width of wheel" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter Real hollowFraction = 0.8 "1.0: Completely hollow, 0.0: rigid cylinder" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter UWmBody.UWTypes.Color wheelColor = {30, 30, 30} "Color of wheel" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  UWmBody.UWParts.Body body(final r_CM = {0, 0, 0}, final m = wheelMass, final I_11 = wheel_I_long, final I_22 = wheel_I_axis, final I_33 = wheel_I_long, final I_21 = 0, final I_31 = 0, final I_32 = 0, animation = false) annotation(Placement(transformation(extent = {{20, -10}, {40, 10}})));
  UWmBody.UWInterfaces.Frame_a frame_a "Frame fixed in wheel center point (y-axis: along wheel axis, z-axis: upwards)" annotation(Placement(transformation(extent = {{-16, -16}, {16, 16}})));
  UWmBody.UWJoints.RollingWheel rollingWheel(wheelRadius = wheelRadius, stateSelect = StateSelect.avoid) annotation(Placement(transformation(extent = {{-60, -60}, {-40, -40}})));
  UWmBody.UWVisualizers.FixedShape fixedShape(final animation = animation, final r_shape = {0, -width, 0}, final lengthDirection = {0, 1, 0}, final widthDirection = {1, 0, 0}, final length = 2 * width, final width = 2 * wheelRadius, final height = 2 * wheelRadius, final color = wheelColor, final extra = hollowFraction, final shapeType = "pipe") if animation annotation(Placement(transformation(extent = {{20, 20}, {40, 40}})));
equation
  rollingWheel.x = x;
  rollingWheel.y = y;
  rollingWheel.angles = angles;
  rollingWheel.der_angles = der_angles;
  connect(body.frame_a, frame_a) annotation(Line(points = {{20, 0}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(rollingWheel.frame_a, frame_a) annotation(Line(points = {{-50, -50}, {-25, -50}, {-25, 0}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(fixedShape.frame_a, frame_a) annotation(Line(points = {{20, 30}, {0, 30}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));
  annotation(defaultComponentName = "wheel", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, fillColor = {175, 175, 175}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, -80}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 85}, {150, 125}}, textString = "%name"), Ellipse(visible = true, rotation = 45, lineColor = {128, 128, 128}, fillColor = {230, 230, 230}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-80, -80}, {80, 80}}), Ellipse(visible = true, rotation = 45, lineColor = {128, 128, 128}, fillColor = {255, 255, 255}, extent = {{-80, -80}, {80, 80}}), Ellipse(visible = true, rotation = 45, lineColor = {64, 64, 64}, fillColor = {230, 230, 230}, pattern = LinePattern.None, fillPattern = FillPattern.HorizontalCylinder, extent = {{-50, -50}, {50, 50}})}));
end RollingWheel;
