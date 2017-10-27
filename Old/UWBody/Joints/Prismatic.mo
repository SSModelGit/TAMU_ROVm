within UWBody.Joints;

model Prismatic "Prismatic joint (1 translational degree-of-freedom, 2 potential states, optional axis flange)"
  extends UWBody.Interfaces.PartialElementaryJoint;
  Modelica.Mechanics.Translational.Interfaces.Flange_a axis if useAxisFlange "1-dim. translational flange that drives the joint" annotation(Placement(transformation(extent = {{90, 50}, {70, 70}})));
  Modelica.Mechanics.Translational.Interfaces.Flange_b support if useAxisFlange "1-dim. translational flange of the drive drive support (assumed to be fixed in the world frame, NOT in the joint)" annotation(Placement(transformation(extent = {{-30, 50}, {-50, 70}})));
  parameter Boolean useAxisFlange = false "= true, if axis flange is enabled" annotation(Evaluate = true, HideResult = true, choices(checkBox = true));
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter UWBody.Types.Axis n = {1, 0, 0} "Axis of translation resolved in frame_a (= same as in frame_b)" annotation(Evaluate = true);
  constant SI.Position s_offset = 0 "Relative distance offset (distance between frame_a and frame_b = s_offset + s)" annotation(Evaluate = false);
  parameter Types.Axis boxWidthDirection = {0, 1, 0} "Vector in width direction of box, resolved in frame_a" annotation(Evaluate = true, Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Distance boxWidth = world.defaultJointWidth "Width of prismatic joint box" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Distance boxHeight = boxWidth "Height of prismatic joint box" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input Types.Color boxColor = UWBody.Types.Defaults.JointColor "Color of prismatic joint box" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter StateSelect stateSelect = StateSelect.prefer "Priority to use distance s and v=der(s) as states" annotation(Dialog(tab = "Advanced"));
  final parameter Real e[3](each final unit = "1") = Modelica.Math.Vectors.normalizeWithAssert(n) "Unit vector in direction of prismatic axis n";
  SI.Position s(start = 0, final stateSelect = stateSelect) "Relative distance between frame_a and frame_b" annotation(unassignedMessage = "
     The relative distance s of a prismatic joint cannot be determined.
     Possible reasons:
     - A non-zero mass might be missing on either side of the parts
       connected to the prismatic joint.
     - Too many StateSelect.always are defined and the model
       has less degrees of freedom as specified with this setting
       (remove all StateSelect.always settings).
     ");
  Modelica.SIunits.Velocity v(start = 0, final stateSelect = stateSelect) "First derivative of s (relative velocity)";
  Modelica.SIunits.Acceleration a(start = 0) "Second derivative of s (relative acceleration)";
  SI.Force f "Actuation force in direction of joint axis";
protected
  Visualizers.Advanced.Shape box(shapeType = "box", color = boxColor, specularCoefficient = specularCoefficient, length = if noEvent(abs(s + s_offset) > 1.e-6) then s + s_offset else 1.e-6, width = boxWidth, height = boxHeight, lengthDirection = e, widthDirection = boxWidthDirection, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
  Translational.Components.Fixed fixed annotation(Placement(transformation(extent = {{-50, 30}, {-30, 50}})));
  Translational.Interfaces.InternalSupport internalAxis(f = f) annotation(Placement(transformation(extent = {{70, 50}, {90, 30}})));
  Translational.Sources.ConstantForce constantForce(f_constant = 0) if not useAxisFlange annotation(Placement(transformation(extent = {{40, 30}, {60, 50}})));
equation
  v = der(s);
  a = der(v);
  // relationships between kinematic quantities of frame_a and of frame_b
  frame_b.r_0 = frame_a.r_0 + Frames.resolve1(frame_a.R, e * (s_offset + s));
  frame_b.R = frame_a.R;
  // Force and torque balance
  zeros(3) = frame_a.f + frame_b.f;
  zeros(3) = frame_a.t + frame_b.t + cross(e * (s_offset + s), frame_b.f);
  // d'Alemberts principle
  f = -e * frame_b.f;
  // Connection to internal connectors
  s = internalAxis.s;
  connect(fixed.flange, support) annotation(Line(points = {{-40, 40}, {-40, 60}}, color = {0, 127, 0}));
  connect(internalAxis.flange, axis) annotation(Line(points = {{80, 40}, {80, 60}}, color = {0, 127, 0}));
  connect(constantForce.flange, internalAxis.flange) annotation(Line(points = {{60, 40}, {80, 40}}, color = {0, 127, 0}));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {2.5, 2.5}), graphics = {Line(visible = true, points = {{100, -30}, {100, 21}}), Text(visible = useAxisFlange, textColor = {64, 64, 64}, extent = {{-150, -135}, {150, -95}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -90}, {150, -60}}, textString = "n=%n"), Rectangle(visible = useAxisFlange, lineColor = {0, 0, 255}, fillColor = {192, 192, 192}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{90, 30}, {100, 70}}), Text(visible = not useAxisFlange, textColor = {64, 64, 64}, extent = {{-150, 60}, {150, 100}}, textString = "%name"), Polygon(visible = true, origin = {-32.5, 5}, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-7.5, 45}, {7.5, 52.5}, {7.5, -42.5}, {-7.5, -55}}), Rectangle(visible = true, origin = {-14.286, -1.648}, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-85.714, -48.352}, {-25.714, 51.648}}), Polygon(visible = true, origin = {-65, 55}, fillColor = {217, 217, 217}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-35, -5}, {-35, -5}, {-35, 2.5}, {40, 2.5}, {25, -5}}), Line(visible = true, origin = {-40, 0}, points = {{15, 57.5}, {0, 50}, {-60, 50}, {0, 50}, {0, -50}}, color = {255, 255, 255}), Polygon(visible = true, origin = {-55.276, 7.5}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, points = {{-44.724, 42.5}, {-44.724, 50}, {30.276, 50}, {30.276, -45}, {15.276, -57.5}, {-44.724, -57.5}}), Rectangle(visible = true, origin = {-3.846, 1.2}, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-31.154, -31.2}, {103.846, 28.8}}), Polygon(visible = true, origin = {34.375, 32.5}, lineColor = {64, 64, 64}, fillColor = {217, 217, 217}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-69.375, -2.5}, {-61.875, 2.5}, {65.625, 2.5}, {65.625, -2.5}}), Line(visible = true, origin = {32.5, 30}, points = {{-67.5, 0}, {67.5, 0}}, color = {255, 255, 255}), Polygon(visible = true, origin = {20.5, 8}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, points = {{-55.5, 22}, {-48, 27}, {79.5, 27}, {79.5, -38}, {-55.5, -38}}), Line(visible = true, origin = {20, 0}, points = {{-40, 0}, {40, 0}}, color = {64, 64, 64}, arrow = {Arrow.Filled, Arrow.Filled}, arrowSize = 20), Text(visible = true, textColor = {128, 128, 128}, extent = {{60, -13}, {96, 12}}, textString = "b"), Text(visible = true, textColor = {128, 128, 128}, extent = {{-95, -9}, {-60, 13}}, textString = "a")}), Documentation(info = "<html>
<p>
Joint where frame_b is translated along axis n which is fixed in frame_a.
The two frames coincide when the relative distance \"s = 0\".
</p>

<p>
Optionally, two additional 1-dimensional mechanical flanges
(flange \"axis\" represents the driving flange and
flange \"support\" represents the bearing) can be enabled via
parameter <b>useAxisFlange</b>. The enabled axis flange can be
driven with elements of the
<a href=\"modelica://Modelica.Mechanics.Translational\">Modelica.Mechanics.Translational</a>
library.

</p>

<p>
In the \"Advanced\" menu it can be defined via parameter <b>stateSelect</b>
that the relative distance \"s\" and its derivative shall be definitely
used as states by setting stateSelect=StateSelect.always.
Default is StateSelect.prefer to use the relative distance and its
derivative as preferred states. The states are usually selected automatically.
In certain situations, especially when closed kinematic loops are present,
it might be slightly more efficient, when using the StateSelect.always setting.
</p>

<p>
In the following figure the animation of a prismatic
joint is shown. The light blue coordinate system is
frame_a and the dark blue coordinate system is
frame_b of the joint. The black arrow is parameter
vector \"n\" defining the translation axis
(here: n = {1,1,0}).
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/Prismatic.png\">
</p>

</html>"));
end Prismatic;
