within UWBody.Sensors;

model RelativeSensor "Measure relative kinematic quantities between two frame connectors"
  extends UWBody.Sensors.Internal.PartialRelativeSensor;
  Interfaces.Frame_resolve frame_resolve if resolveInFrame == UWBody.Types.ResolveInFrameAB.frame_resolve or resolveInFrameAfterDifferentiation == UWBody.Types.ResolveInFrameAB.frame_resolve "If resolveInFrame = Types.ResolveInFrameAB.frame_resolve, the output signals are resolved in this frame" annotation(Placement(transformation(extent = {{84, 64}, {116, 96}}), iconTransformation(extent = {{84, 64}, {116, 96}})));
  parameter Boolean animation = true "= true, if animation shall be enabled (show arrow)";
  parameter UWBody.Types.ResolveInFrameAB resolveInFrame = UWBody.Types.ResolveInFrameAB.frame_a "Frame in which vectors are resolved before differentiation (world, frame_a, frame_b, or frame_resolve)";
  parameter Boolean get_r_rel = false "= true, to measure the relative position vector from the origin of frame_a to frame_b" annotation(HideResult = true, choices(checkBox = true));
  parameter Boolean get_v_rel = false "= true, to measure the relative velocity of the origin of frame_b with respect to frame_a" annotation(HideResult = true, choices(checkBox = true));
  parameter Boolean get_a_rel = false "= true, to measure the relative acceleration of the origin of frame_b with respect to frame_a" annotation(HideResult = true, choices(checkBox = true));
  parameter Boolean get_w_rel = false "= true, to measure the relative angular velocity of frame_b with respect to frame_a" annotation(HideResult = true, choices(checkBox = true));
  parameter Boolean get_z_rel = false "= true, to measure the relative angular acceleration of frame_b with respect to frame_a" annotation(HideResult = true, choices(checkBox = true));
  parameter Boolean get_angles = false "= true, to measure the 3 rotation angles" annotation(HideResult = true, choices(checkBox = true), Dialog(group = "3 angles to rotate frame_a into frame_b along the axes defined in \"sequence\""));
  parameter Types.RotationSequence sequence(min = {1, 1, 1}, max = {3, 3, 3}) = {1, 2, 3} "If get_angles=true: Angles are returned to rotate frame_a around axes sequence[1], sequence[2] and finally sequence[3] into frame_b" annotation(HideResult = true, Evaluate = true, Dialog(group = "3 angles to rotate frame_a into frame_b along the axes defined in \"sequence\"", enable = get_angles));
  parameter Modelica.SIunits.Angle guessAngle1 = 0 "If get_angles=true: Select angles[1] such that abs(angles[1] - guessAngle1) is a minimum" annotation(HideResult = true, Dialog(group = "3 angles to rotate frame_a into frame_b along the axes defined in \"sequence\"", enable = get_angles));
  input SI.Diameter arrowDiameter = world.defaultArrowDiameter "Diameter of relative arrow from frame_a to frame_b" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input Types.Color arrowColor = UWBody.Types.Defaults.SensorColor "Color of relative arrow from frame_a to frame_b" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter UWBody.Types.ResolveInFrameAB resolveInFrameAfterDifferentiation = resolveInFrame "Frame in which vectors are resolved after differentiation (world, frame_a, frame_b, or frame_resolve)" annotation(Dialog(tab = "Advanced", group = "if get_v_rel or get_a_rel or get_z_rel", enable = get_v_rel or get_a_rel or get_z_rel));
  Modelica.Blocks.Interfaces.RealOutput r_rel[3](each final quantity = "Length", each final unit = "m") if get_r_rel "Relative position vector frame_b.r_0 - frame_a.r_0 resolved in frame defined by resolveInFrame" annotation(Placement(transformation(origin = {-100, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput v_rel[3](each final quantity = "Velocity", each final unit = "m/s") if get_v_rel "Relative velocity vector" annotation(Placement(transformation(origin = {-60, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput a_rel[3](each final quantity = "Acceleration", each final unit = "m/s2") if get_a_rel "Relative acceleration vector" annotation(Placement(transformation(origin = {-20, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput angles[3](each final quantity = "Angle", each final unit = "rad", each displayUnit = "deg") if get_angles "Angles to rotate frame_a into frame_b via 'sequence'" annotation(Placement(transformation(origin = {20, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput w_rel[3](each final quantity = "AngularVelocity", each final unit = "1/s") if get_w_rel "Relative angular velocity vector" annotation(Placement(transformation(origin = {60, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput z_rel[3](each final quantity = "AngularAcceleration", each final unit = "1/s2") if get_z_rel "Relative angular acceleration vector" annotation(Placement(transformation(origin = {100, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
protected
  RelativePosition relativePosition(resolveInFrame = resolveInFrame) if get_r_rel or get_v_rel or get_a_rel annotation(Placement(transformation(extent = {{-80, -10}, {-60, 10}})));
protected
  Modelica.Blocks.Continuous.Der der1[3] if get_v_rel or get_a_rel annotation(Placement(transformation(extent = {{-10, -10}, {0, 0}}, rotation = -90, origin = {-55, -30})));
  Modelica.Blocks.Continuous.Der der2[3] if get_a_rel annotation(Placement(transformation(extent = {{0, 0}, {10, 10}}, rotation = -90, origin = {-25, -40})));
  UWBody.Sensors.RelativeAngles relativeAngles(sequence = sequence, guessAngle1 = guessAngle1) if get_angles annotation(Placement(transformation(extent = {{10, -25}, {30, -5}})));
  RelativeAngularVelocity relativeAngularVelocity(resolveInFrame = resolveInFrame) if get_w_rel or get_z_rel annotation(Placement(transformation(extent = {{50, -40}, {70, -20}})));
protected
  Modelica.Blocks.Continuous.Der der3[3] if get_z_rel annotation(Placement(transformation(extent = {{-10, -10}, {0, 0}}, rotation = -90, origin = {105, -60})));
  Internal.ZeroForceAndTorque zeroForce1 annotation(Placement(transformation(extent = {{-81, 40}, {-61, 60}})));
  Internal.ZeroForceAndTorque zeroForce2 annotation(Placement(transformation(extent = {{70, 20}, {50, 40}})));
  Internal.ZeroForceAndTorque zeroForce3 if resolveInFrame == UWBody.Types.ResolveInFrameAB.frame_resolve annotation(Placement(transformation(extent = {{70, 50}, {50, 70}})));
protected
  UWBody.Sensors.TransformRelativeVector transformVector_v_rel(frame_r_in = resolveInFrame, frame_r_out = resolveInFrameAfterDifferentiation) if get_v_rel annotation(Placement(transformation(extent = {{-70, -64}, {-50, -44}})));
  UWBody.Sensors.TransformRelativeVector transformVector_a_rel(frame_r_in = resolveInFrame, frame_r_out = resolveInFrameAfterDifferentiation) if get_a_rel annotation(Placement(transformation(extent = {{-30, -80}, {-10, -60}})));
  UWBody.Sensors.TransformRelativeVector transformVector_z_rel(frame_r_in = resolveInFrame, frame_r_out = resolveInFrameAfterDifferentiation) if get_z_rel annotation(Placement(transformation(extent = {{90, -95}, {110, -75}})));
protected
  outer UWBody.World world;
  UWBody.Visualizers.Advanced.Arrow arrow(r = frame_a.r_0, r_head = frame_b.r_0 - frame_a.r_0, diameter = arrowDiameter, color = arrowColor, specularCoefficient) if world.enableAnimation and animation;
equation
  connect(relativePosition.frame_a, frame_a) annotation(Line(points = {{-80, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(relativePosition.frame_b, frame_b) annotation(Line(points = {{-60, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(relativePosition.r_rel, r_rel) annotation(Line(points = {{-70, -11}, {-70, -15}, {-80, -15}, {-80, -80}, {-100, -80}, {-100, -110}}, color = {0, 0, 127}));
  connect(zeroForce1.frame_a, frame_a) annotation(Line(points = {{-81, 50}, {-90, 50}, {-90, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(zeroForce2.frame_a, frame_b) annotation(Line(points = {{70, 30}, {90, 30}, {90, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(relativePosition.r_rel, der1.u) annotation(Line(points = {{-70, -11}, {-70, -15}, {-60, -15}, {-60, -19}}, color = {0, 0, 127}));
  connect(der2.u, der1.y) annotation(Line(points = {{-20, -39}, {-20, -35}, {-60, -35}, {-60, -30.5}}, color = {0, 0, 127}));
  connect(relativeAngles.frame_a, frame_a) annotation(Line(points = {{10, -15}, {0, -15}, {0, 30}, {-90, 30}, {-90, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(relativeAngles.frame_b, frame_b) annotation(Line(points = {{30, -15}, {40, -15}, {40, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(relativeAngles.angles, angles) annotation(Line(points = {{20, -26}, {20, -110}}, color = {0, 0, 127}));
  connect(relativeAngularVelocity.frame_b, frame_b) annotation(Line(points = {{70, -30}, {80, -30}, {80, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(relativeAngularVelocity.frame_a, frame_a) annotation(Line(points = {{50, -30}, {0, -30}, {0, 30}, {-90, 30}, {-90, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(relativeAngularVelocity.w_rel, w_rel) annotation(Line(points = {{60, -41}, {60, -110}}, color = {0, 0, 127}));
  connect(relativeAngularVelocity.w_rel, der3.u) annotation(Line(points = {{60, -41}, {60, -43}, {100, -43}, {100, -49}}, color = {0, 0, 127}));
  connect(der1.y, transformVector_v_rel.r_in) annotation(Line(points = {{-60, -30.5}, {-60, -42}}, color = {0, 0, 127}));
  connect(transformVector_v_rel.r_out, v_rel) annotation(Line(points = {{-60, -65}, {-60, -110}}, color = {0, 0, 127}));
  connect(transformVector_v_rel.frame_a, frame_a) annotation(Line(points = {{-70, -54}, {-90, -54}, {-90, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(transformVector_v_rel.frame_b, frame_b) annotation(Line(points = {{-50, -54}, {-36, -54}, {-36, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(transformVector_v_rel.frame_resolve, frame_resolve) annotation(Line(points = {{-50, -45.9}, {-47, -45.9}, {-47, -47}, {-42, -47}, {-42, 80}, {100, 80}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(frame_resolve, relativePosition.frame_resolve) annotation(Line(points = {{100, 80}, {-50, 80}, {-50, 8.1}, {-60, 8.1}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(frame_resolve, zeroForce3.frame_a) annotation(Line(points = {{100, 80}, {80, 80}, {80, 60}, {70, 60}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(relativeAngularVelocity.frame_resolve, frame_resolve) annotation(Line(points = {{70, -21.9}, {77, -21.9}, {77, 80}, {100, 80}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(der2.y, transformVector_a_rel.r_in) annotation(Line(points = {{-20, -50.5}, {-20, -58}}, color = {0, 0, 127}));
  connect(transformVector_a_rel.frame_a, frame_a) annotation(Line(points = {{-30, -70}, {-90, -70}, {-90, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(transformVector_a_rel.frame_b, frame_b) annotation(Line(points = {{-10, -70}, {6, -70}, {6, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(transformVector_a_rel.frame_resolve, frame_resolve) annotation(Line(points = {{-10, -61.9}, {-10, 80}, {100, 80}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(transformVector_a_rel.r_out, a_rel) annotation(Line(points = {{-20, -81}, {-20, -110}}, color = {0, 0, 127}));
  connect(der3.y, transformVector_z_rel.r_in) annotation(Line(points = {{100, -60.5}, {100, -73}}, color = {0, 0, 127}));
  connect(transformVector_z_rel.r_out, z_rel) annotation(Line(points = {{100, -96}, {100, -110}}, color = {0, 0, 127}));
  connect(transformVector_z_rel.frame_a, frame_a) annotation(Line(points = {{90, -85}, {-90, -85}, {-90, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(transformVector_z_rel.frame_b, frame_b) annotation(Line(points = {{110, -85}, {119, -85}, {119, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(transformVector_z_rel.frame_resolve, frame_resolve) annotation(Line(points = {{110, -76.9}, {116, -76.9}, {116, 80}, {100, 80}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = get_r_rel, points = {{-84, 0}, {-84, -60}, {-100, -60}, {-100, -100}}, color = {0, 0, 127}), Line(visible = get_a_rel, points = {{-20, -67}, {-20, -100}}, color = {0, 0, 127}), Line(visible = get_w_rel, points = {{60, -36}, {60, -100}}, color = {0, 0, 127}), Line(visible = get_z_rel, points = {{86, 0}, {86, -60}, {100, -60}, {100, -100}}, color = {0, 0, 127}), Line(visible = get_v_rel, points = {{-60, -36}, {-60, -100}}, color = {0, 0, 127}), Line(visible = get_angles, points = {{20, -67}, {20, -100}}, color = {0, 0, 127}), Text(visible = true, origin = {0, -10}, textColor = {64, 64, 64}, extent = {{-130, 90}, {130, 130}}, textString = "%name"), Text(visible = get_r_rel, extent = {{-130, -90}, {-95, -74}}, textString = "r"), Text(visible = get_v_rel, extent = {{-95, -90}, {-60, -74}}, textString = "v"), Text(visible = get_a_rel, extent = {{-55, -90}, {-20, -74}}, textString = "a"), Text(visible = get_angles, extent = {{-71, -54}, {96, -35}}, textString = "angles"), Text(visible = get_w_rel, extent = {{63, -90}, {103, -73}}, textString = "w"), Text(visible = get_z_rel, extent = {{103, -87}, {149, -71}}, textString = "z")}), Documentation(info = "<html>
<p>
Relative kinematic quantities between frame_a and frame_b are
determined and provided at the conditional output signal connectors.
For example, if parameter \"get_r_rel = <b>true</b>\", the connector
\"r_rel\" is enabled and contains the relative vector from
frame_a to frame_b. The following quantities can be provided
as output signals:
</p>

<ol>
<li> Relative position vector (= r_rel) </li>
<li> Relative velocity vector (= v_rel)</li>
<li> Relative acceleration vector (= a_rel)</li>
<li> Three angles to rotate frame_a into frame_b (= angles)</li>
<li> Relative angular velocity vector (= w_rel)</li>
<li> Relative angular acceleration vector (= z_rel)</li>
</ol>

<p>
Via parameter <b>resolveInFrame</b> it is defined, in which frame
a vector is resolved (before differentiation):
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>resolveInFrame =<br>Types.ResolveInFrameAB.</b></th><th><b>Meaning</b></th></tr>
<tr><td valign=\"top\">world</td>
    <td valign=\"top\">Resolve vectors in world frame</td></tr>

<tr><td valign=\"top\">frame_a</td>
    <td valign=\"top\">Resolve vectors in frame_a</td></tr>

<tr><td valign=\"top\">frame_b</td>
    <td valign=\"top\">Resolve vectors in frame_b</td></tr>

<tr><td valign=\"top\">frame_resolve</td>
    <td valign=\"top\">Resolve vectors in frame_resolve</td></tr>
</table>

<p>
If resolveInFrame = Types.ResolveInFrameAB.frame_resolve, the conditional connector
\"frame_resolve\" is enabled and the vectors are resolved in the frame, to
which frame_resolve is connected. Note, if this connector is enabled, it must
be connected.
</p>

<p>
In the following figure the animation of a RelativeSensor
component is shown. The light blue coordinate system is
frame_a, the dark blue coordinate system is frame_b, and
the yellow arrow is the animated sensor.
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Sensors/RelativeSensor.png\">
</p>

<p>
Note, derivatives
of relative kinematic quantities are always performed with
respect to the frame, in which the vector to be differentiated
is resolved. After differentiation, it is possible via parameter
<b>resolveInFrameAfterDifferentiation</b> (in the \"Advanced\" menu)
to resolve the differentiated
vector in another frame.
</p>
<p>
For example, if resolveInFrame = <b>Types.ResolveInFrameAB.frame_b</b>, then
</p>

<pre>
   r_rel = resolve2(frame_b.R, frame_b.r_0 - frame_a.r0);
   v_rel = <b>der</b>(r_rel);
</pre>

<p>
is returned (r_rel = resolve2(frame_b.R, frame_b.r_0 - frame_a.r0)), i.e.,
the derivative of the relative distance from frame_a to frame_b,
resolved in frame_b. If
<b>resolveInFrameAfterDifferentiation</b> = Types.ResolveInFrameAB.world, then
v_rel is additionally transformed to:
</p>

<pre>
   v_rel = resolve1(frame_b.R, <b>der</b>(r_rel))
</pre>

<p>
The cut-force and the cut-torque in frame_resolve are
always zero, whether frame_resolve is connected or not.
</p>

<p>
If <b>get_angles</b> = <b>true</b>, the 3 angles to rotate frame_a
into frame_b along the axes defined by parameter <b>sequence</b>
are returned. For example, if sequence = {3,1,2} then frame_a is
rotated around angles[1] along the z-axis, afterwards it is rotated
around angles[2] along the x-axis, and finally it is rotated around
angles[3] along the y-axis and is then identical to frame_b.
The 3 angles are returned in the range
</p>
<pre>
    -<font face=\"Symbol\">p</font> &lt;= angles[i] &lt;= <font face=\"Symbol\">p</font>
</pre>
<p>
There are <b>two solutions</b> for \"angles[1]\" in this range.
Via parameter <b>guessAngle1</b> (default = 0) the
returned solution is selected such that |angles[1] - guessAngle1| is
minimal. The relative transformation matrix between frame_a and
frame_b may be in a singular configuration with respect to \"sequence\", i.e.,
there is an infinite number of angle values leading to the same relative
transformation matrix. In this case, the returned solution is
selected by setting angles[1] = guessAngle1. Then angles[2]
and angles[3] can be uniquely determined in the above range.
</p>
<p>
The parameter <b>sequence</b> has the restriction that
only values 1,2,3 can be used and that sequence[1] &ne; sequence[2]
and sequence[2] &ne; sequence[3]. Often used values are:
</p>
<pre>
  sequence = <b>{1,2,3}</b>  // Cardan or Tait-Bryan angle sequence
           = <b>{3,1,3}</b>  // Euler angle sequence
           = <b>{3,2,1}</b>
</pre>
</html>"));
end RelativeSensor;
