within UWmBody.UWJoints;

model FreeMotionScalarInit "Free motion joint with scalar initialization and state selection (6 degrees-of-freedom, 12 potential states)"
  extends UWmBody.UWInterfaces.PartialTwoFrames;
  parameter Boolean animation = true "= true, if animation shall be enabled (show arrow from frame_a to frame_b)" annotation(Dialog(enable = use_r));
  parameter Boolean use_r = false "= true, if r_rel_a shall be used" annotation(Evaluate = true, HideResult = true, Dialog(tab = "Translational Initialization", group = "Position vector r_rel_a from origin of frame_a to origin of frame_b, resolved in frame_a"));
  Modelica.Blocks.Interfaces.RealOutput r_rel_a_1(final quantity = "Length", final unit = "m", start = 0, final stateSelect = r_rel_a_1_stateSelect) if use_r "Relative distance r_rel_a[1]" annotation(Dialog(enable = use_r, tab = "Translational Initialization", group = "Position vector r_rel_a from origin of frame_a to origin of frame_b, resolved in frame_a", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput r_rel_a_2(final quantity = "Length", final unit = "m", start = 0, final stateSelect = r_rel_a_2_stateSelect) if use_r "Relative distance r_rel_a[2]" annotation(Dialog(enable = use_r, tab = "Translational Initialization", group = "Position vector r_rel_a from origin of frame_a to origin of frame_b, resolved in frame_a", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput r_rel_a_3(final quantity = "Length", final unit = "m", start = 0, final stateSelect = r_rel_a_3_stateSelect) if use_r "Relative distance r_rel_a[3]" annotation(Dialog(enable = use_r, tab = "Translational Initialization", group = "Position vector r_rel_a from origin of frame_a to origin of frame_b, resolved in frame_a", showStartAttribute = true));
  parameter StateSelect r_rel_a_1_stateSelect = StateSelect.never "StateSelect of r_rel_a[1]" annotation(HideResult = true, Dialog(enable = use_r, tab = "Translational Initialization", group = "Position vector r_rel_a from origin of frame_a to origin of frame_b, resolved in frame_a"));
  parameter StateSelect r_rel_a_2_stateSelect = StateSelect.never "StateSelect of r_rel_a[2]" annotation(HideResult = true, Dialog(enable = use_r, tab = "Translational Initialization", group = "Position vector r_rel_a from origin of frame_a to origin of frame_b, resolved in frame_a"));
  parameter StateSelect r_rel_a_3_stateSelect = StateSelect.never "StateSelect of r_rel_a[3]" annotation(HideResult = true, Dialog(enable = use_r, tab = "Translational Initialization", group = "Position vector r_rel_a from origin of frame_a to origin of frame_b, resolved in frame_a"));
  parameter Boolean use_v = false "= true, if v_rel_a shall be used" annotation(Evaluate = true, HideResult = true, Dialog(enable = use_r, tab = "Translational Initialization", group = "Velocity vector v_rel_a = der(r_rel_a)"));
  Modelica.Blocks.Interfaces.RealOutput v_rel_a_1(final quantity = "Velocity", final unit = "m/s", start = 0, final stateSelect = v_rel_a_1_stateSelect) if use_r and use_v "Relative velocity v_rel_a[1]" annotation(Dialog(enable = use_r and use_v, tab = "Translational Initialization", group = "Velocity vector v_rel_a = der(r_rel_a)", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput v_rel_a_2(final quantity = "Velocity", final unit = "m/s", start = 0, final stateSelect = v_rel_a_2_stateSelect) if use_r and use_v "Relative velocity v_rel_a[2]" annotation(Dialog(enable = use_r and use_v, tab = "Translational Initialization", group = "Velocity vector v_rel_a = der(r_rel_a)", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput v_rel_a_3(final quantity = "Velocity", final unit = "m/s", start = 0, final stateSelect = v_rel_a_3_stateSelect) if use_r and use_v "Relative velocity v_rel_a[3]" annotation(Dialog(enable = use_r and use_v, tab = "Translational Initialization", group = "Velocity vector v_rel_a = der(r_rel_a)", showStartAttribute = true));
  parameter StateSelect v_rel_a_1_stateSelect = StateSelect.never "StateSelect of v_rel_a[1]" annotation(HideResult = true, Dialog(enable = use_r and use_v, tab = "Translational Initialization", group = "Velocity vector v_rel_a = der(r_rel_a)"));
  parameter StateSelect v_rel_a_2_stateSelect = StateSelect.never "StateSelect of v_rel_a[2]" annotation(HideResult = true, Dialog(enable = use_r and use_v, tab = "Translational Initialization", group = "Velocity vector v_rel_a = der(r_rel_a)"));
  parameter StateSelect v_rel_a_3_stateSelect = StateSelect.never "StateSelect of v_rel_a[3]" annotation(HideResult = true, Dialog(enable = use_r and use_v, tab = "Translational Initialization", group = "Velocity vector v_rel_a = der(r_rel_a)"));
  parameter Boolean use_a = false "= true, if a_rel_a shall be used" annotation(Evaluate = true, HideResult = true, Dialog(enable = use_r and use_v, tab = "Translational Initialization", group = "Acceleration vector a_rel_a = der(v_rel_a)"));
  Modelica.Blocks.Interfaces.RealOutput a_rel_a_1(final quantity = "Acceleration", final unit = "m/s2", start = 0) if use_r and use_v and use_a "Relative acceleration a_rel_a[1]" annotation(Dialog(enable = use_r and use_v and use_a, tab = "Translational Initialization", group = "Acceleration vector a_rel_a = der(v_rel_a)", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput a_rel_a_2(final quantity = "Acceleration", final unit = "m/s2", start = 0) if use_r and use_v and use_a "Relative acceleration a_rel_a[2]" annotation(Dialog(enable = use_r and use_v and use_a, tab = "Translational Initialization", group = "Acceleration vector a_rel_a = der(v_rel_a)", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput a_rel_a_3(final quantity = "Acceleration", final unit = "m/s2", start = 0) if use_r and use_v and use_a "Relative acceleration a_rel_a[3]" annotation(Dialog(enable = use_r and use_v and use_a, tab = "Translational Initialization", group = "Acceleration vector a_rel_a = der(v_rel_a)", showStartAttribute = true));
  parameter Boolean use_angle = false "= true, if angle shall be used" annotation(Evaluate = true, HideResult = true, Dialog(tab = "Angle Initialization", group = "Angles to rotate frame_a to frame_b along sequence_start"));
  parameter UWTypes.RotationSequence sequence_start = {1, 2, 3} "Sequence of angle rotations" annotation(Evaluate = true, Dialog(enable = use_angle, tab = "Angle Initialization", group = "Angles to rotate frame_a to frame_b along sequence_start"));
  Modelica.Blocks.Interfaces.RealOutput angle_1(final quantity = "Angle", final unit = "rad", start = 0, stateSelect = angle_1_stateSelect) if use_angle "First rotation angle or dummy" annotation(Dialog(enable = use_angle, tab = "Angle Initialization", group = "Angles to rotate frame_a to frame_b along sequence_start", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput angle_2(final quantity = "Angle", final unit = "rad", start = 0, stateSelect = angle_2_stateSelect) if use_angle "Second rotation angle or dummy" annotation(Dialog(enable = use_angle, tab = "Angle Initialization", group = "Angles to rotate frame_a to frame_b along sequence_start", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput angle_3(final quantity = "Angle", final unit = "rad", start = 0, stateSelect = angle_3_stateSelect) if use_angle "Third rotation angle or dummy" annotation(Dialog(enable = use_angle, tab = "Angle Initialization", group = "Angles to rotate frame_a to frame_b along sequence_start", showStartAttribute = true));
  parameter StateSelect angle_1_stateSelect = StateSelect.never "StateSelect of angle_1" annotation(HideResult = true, Dialog(enable = use_angle, tab = "Angle Initialization", group = "Angles to rotate frame_a to frame_b along sequence_start"));
  parameter StateSelect angle_2_stateSelect = StateSelect.never "StateSelect of angle_2" annotation(HideResult = true, Dialog(enable = use_angle, tab = "Angle Initialization", group = "Angles to rotate frame_a to frame_b along sequence_start"));
  parameter StateSelect angle_3_stateSelect = StateSelect.never "StateSelect of angle_3" annotation(HideResult = true, Dialog(enable = use_angle, tab = "Angle Initialization", group = "Angles to rotate frame_a to frame_b along sequence_start"));
  parameter Boolean use_angle_d = false "= true, if angle_d shall be used" annotation(Evaluate = true, HideResult = true, Dialog(enable = use_angle, tab = "Angle Initialization", group = "angle_d = der(angle)"));
  Modelica.Blocks.Interfaces.RealOutput angle_d_1(final quantity = "AngularVelocity", final unit = "rad/s", start = 0, final stateSelect = angle_d_1_stateSelect) if use_angle and use_angle_d "= der(angle_1)" annotation(Dialog(enable = use_angle and use_angle_d, tab = "Angle Initialization", group = "angle_d = der(angle)", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput angle_d_2(final quantity = "AngularVelocity", final unit = "rad/s", start = 0, final stateSelect = angle_d_2_stateSelect) if use_angle and use_angle_d "= der(angle_2)" annotation(Dialog(enable = use_angle and use_angle_d, tab = "Angle Initialization", group = "angle_d = der(angle)", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput angle_d_3(final quantity = "AngularVelocity", final unit = "rad/s", start = 0, final stateSelect = angle_d_3_stateSelect) if use_angle and use_angle_d "= der(angle_3)" annotation(Dialog(enable = use_angle and use_angle_d, tab = "Angle Initialization", group = "angle_d = der(angle)", showStartAttribute = true));
  parameter StateSelect angle_d_1_stateSelect = StateSelect.never "StateSelect of angle_d_1" annotation(HideResult = true, Dialog(enable = use_angle and use_angle_d, tab = "Angle Initialization", group = "angle_d = der(angle)"));
  parameter StateSelect angle_d_2_stateSelect = StateSelect.never "StateSelect of angle_d_2" annotation(HideResult = true, Dialog(enable = use_angle and use_angle_d, tab = "Angle Initialization", group = "angle_d = der(angle)"));
  parameter StateSelect angle_d_3_stateSelect = StateSelect.never "StateSelect of angle_d_3" annotation(HideResult = true, Dialog(enable = use_angle and use_angle_d, tab = "Angle Initialization", group = "angle_d = der(angle)"));
  parameter Boolean use_angle_dd = false "= true, if angle_dd shall be used" annotation(Evaluate = true, HideResult = true, Dialog(enable = use_angle and use_angle_d, tab = "Angle Initialization", group = "angle_dd = der(angle_d)"));
  Modelica.Blocks.Interfaces.RealOutput angle_dd_1(final quantity = "AngularAcceleration", final unit = "rad/s2", start = 0) if use_angle and use_angle_d and use_angle_dd "= der(angle_d_1)" annotation(Dialog(enable = use_angle and use_angle_d and use_angle_dd, tab = "Angle Initialization", group = "angle_dd = der(angle_d)", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput angle_dd_2(final quantity = "AngularAcceleration", final unit = "rad/s2", start = 0) if use_angle and use_angle_d and use_angle_dd "= der(angle_d_2)" annotation(Dialog(enable = use_angle and use_angle_d and use_angle_dd, tab = "Angle Initialization", group = "angle_dd = der(angle_d)", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput angle_dd_3(final quantity = "AngularAcceleration", final unit = "rad/s2", start = 0) if use_angle and use_angle_d and use_angle_dd "= der(angle_d_3)" annotation(Dialog(enable = use_angle and use_angle_d and use_angle_dd, tab = "Angle Initialization", group = "angle_dd = der(angle_d)", showStartAttribute = true));
  parameter Boolean use_w = false "= true, if w_rel_b shall be used" annotation(Evaluate = true, HideResult = true, Dialog(tab = "Angular Velocity Initialization", group = "Angular velocity w_rel_b of frame_b with respect to frame_a, resolved in frame_b"));
  Modelica.Blocks.Interfaces.RealOutput w_rel_b_1(final quantity = "AngularVelocity", final unit = "rad/s", start = 0, stateSelect = w_rel_b_1_stateSelect) if use_w "Relative angular velocity w_rel_b[1]" annotation(Dialog(enable = use_w, tab = "Angular Velocity Initialization", group = "Angular velocity w_rel_b of frame_b with respect to frame_a, resolved in frame_b", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput w_rel_b_2(final quantity = "AngularVelocity", final unit = "rad/s", start = 0, stateSelect = w_rel_b_2_stateSelect) if use_w "Relative angular velocity w_rel_b[2]" annotation(Dialog(enable = use_w, tab = "Angular Velocity Initialization", group = "Angular velocity w_rel_b of frame_b with respect to frame_a, resolved in frame_b", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput w_rel_b_3(final quantity = "AngularVelocity", final unit = "rad/s", start = 0, stateSelect = w_rel_b_3_stateSelect) if use_w "Relative angular velocity w_rel_b[3]" annotation(Dialog(enable = use_w, tab = "Angular Velocity Initialization", group = "Angular velocity w_rel_b of frame_b with respect to frame_a, resolved in frame_b", showStartAttribute = true));
  parameter StateSelect w_rel_b_1_stateSelect = StateSelect.never "StateSelect of w_rel_b[1]" annotation(HideResult = true, Dialog(enable = use_w, tab = "Angular Velocity Initialization", group = "Angular velocity w_rel_b of frame_b with respect to frame_a, resolved in frame_b"));
  parameter StateSelect w_rel_b_2_stateSelect = StateSelect.never "StateSelect of w_rel_b[2]" annotation(HideResult = true, Dialog(enable = use_w, tab = "Angular Velocity Initialization", group = "Angular velocity w_rel_b of frame_b with respect to frame_a, resolved in frame_b"));
  parameter StateSelect w_rel_b_3_stateSelect = StateSelect.never "StateSelect of w_rel_b[3]" annotation(HideResult = true, Dialog(enable = use_w, tab = "Angular Velocity Initialization", group = "Angular velocity w_rel_b of frame_b with respect to frame_a, resolved in frame_b"));
  parameter Boolean use_z = false "= true, if z_rel_b shall be used" annotation(Evaluate = true, HideResult = true, Dialog(enable = use_w, tab = "Angular Velocity Initialization", group = "Angular acceleration z_rel_b = der(w_rel_b)"));
  Modelica.Blocks.Interfaces.RealOutput z_rel_b_1(final quantity = "AngularAcceleration", final unit = "rad/s2", start = 0) if use_w and use_z "Relative angular acceleration z_rel_b[1]" annotation(Dialog(enable = use_w and use_z, tab = "Angular Velocity Initialization", group = "Angular acceleration z_rel_b = der(w_rel_b)", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput z_rel_b_2(final quantity = "AngularAcceleration", final unit = "rad/s2", start = 0) if use_w and use_z "Relative angular acceleration z_rel_b[2]" annotation(Dialog(enable = use_w and use_z, tab = "Angular Velocity Initialization", group = "Angular acceleration z_rel_b = der(w_rel_b)", showStartAttribute = true));
  Modelica.Blocks.Interfaces.RealOutput z_rel_b_3(final quantity = "AngularAcceleration", final unit = "rad/s2", start = 0) if use_w and use_z "Relative angular acceleration z_rel_b[3]" annotation(Dialog(enable = use_w and use_z, tab = "Angular Velocity Initialization", group = "Angular acceleration z_rel_b = der(w_rel_b)", showStartAttribute = true));
  parameter SI.Length arrowDiameter = world.defaultArrowDiameter "Diameter of arrow from frame_a to frame_b" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation and use_r));
  input UWTypes.Color arrowColor = UWmBody.UWTypes.Defaults.SensorColor "Color of arrow" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation and use_r));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation and use_r));
protected
  UWmBody.UWJoints.Internal.InitPosition initPosition(r_a_0 = frame_a.r_0, r_b_0 = frame_b.r_0, R_a = frame_a.R) if use_r annotation(Placement(transformation(extent = {{-20, 60}, {0, 80}})));
  UWmBody.UWJoints.Internal.InitAngle initAngle(sequence_start = sequence_start) if use_angle annotation(Placement(transformation(extent = {{-60, -10}, {-40, 10}})));
  UWmBody.UWJoints.Internal.InitAngularVelocity initAngularVelocity(R_a = frame_a.R, R_b = frame_b.R) if use_w annotation(Placement(transformation(extent = {{-20, 20}, {0, 40}})));
  Modelica.Blocks.Continuous.Der derv[3] if use_r and use_v annotation(Placement(transformation(extent = {{20, 60}, {40, 80}})));
  Modelica.Blocks.Continuous.Der dera[3] if use_r and use_v and use_a annotation(Placement(transformation(extent = {{60, 60}, {80, 80}})));
  Modelica.Blocks.Continuous.Der derd[3] if use_angle and use_angle_d annotation(Placement(transformation(extent = {{-20, -30}, {0, -10}})));
  Modelica.Blocks.Continuous.Der derdd[3] if use_angle and use_angle_d and use_angle_dd annotation(Placement(transformation(extent = {{20, -30}, {40, -10}})));
  Modelica.Blocks.Continuous.Der derz[3] if use_w and use_z annotation(Placement(transformation(extent = {{20, 20}, {40, 40}})));
  UWmBody.UWSensors.Internal.ZeroForceAndTorque zeroForceAndTorque1 annotation(Placement(transformation(extent = {{-80, -50}, {-60, -30}})));
  UWmBody.UWSensors.Internal.ZeroForceAndTorque zeroForceAndTorque2 annotation(Placement(transformation(extent = {{80, -50}, {60, -30}})));
  UWmBody.UWVisualizers.SignalArrow arrow(diameter = arrowDiameter, color = arrowColor, specularCoefficient = specularCoefficient) if world.enableAnimation and animation and use_r annotation(Placement(transformation(extent = {{-80, 60}, {-60, 80}})));
equation
  // r_rel_a
  connect(initPosition.r_rel_a[1], r_rel_a_1);
  connect(initPosition.r_rel_a[2], r_rel_a_2);
  connect(initPosition.r_rel_a[3], r_rel_a_3);
  // v_rel_a
  connect(derv[1].y, v_rel_a_1);
  connect(derv[2].y, v_rel_a_2);
  connect(derv[3].y, v_rel_a_3);
  // a_rel_a
  connect(dera[1].y, a_rel_a_1);
  connect(dera[2].y, a_rel_a_2);
  connect(dera[3].y, a_rel_a_3);
  // angle
  connect(initAngle.angle[1], angle_1);
  connect(initAngle.angle[2], angle_2);
  connect(initAngle.angle[3], angle_3);
  // angle_d
  connect(derd[1].y, angle_d_1);
  connect(derd[2].y, angle_d_2);
  connect(derd[3].y, angle_d_3);
  // angle_dd
  connect(derdd[1].y, angle_dd_1);
  connect(derdd[2].y, angle_dd_2);
  connect(derdd[3].y, angle_dd_3);
  // w_rel_b
  connect(initAngularVelocity.w_rel_b[1], w_rel_b_1);
  connect(initAngularVelocity.w_rel_b[2], w_rel_b_2);
  connect(initAngularVelocity.w_rel_b[3], w_rel_b_3);
  // z_rel_b
  connect(derz[1].y, z_rel_b_1);
  connect(derz[2].y, z_rel_b_2);
  connect(derz[3].y, z_rel_b_3);
  connect(initPosition.r_rel_a, derv.u) annotation(Line(points = {{1, 70}, {18, 70}}, color = {0, 0, 127}));
  connect(derv.y, dera.u) annotation(Line(points = {{41, 70}, {58, 70}}, color = {0, 0, 127}));
  connect(initAngle.frame_a, frame_a) annotation(Line(points = {{-60, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(initAngle.frame_b, frame_b) annotation(Line(points = {{-40, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(initAngle.angle, derd.u) annotation(Line(points = {{-50, -11}, {-50, -20}, {-22, -20}}, color = {0, 0, 127}));
  connect(derd.y, derdd.u) annotation(Line(points = {{1, -20}, {18, -20}}, color = {0, 0, 127}));
  connect(zeroForceAndTorque1.frame_a, frame_a) annotation(Line(points = {{-80, -40}, {-88, -40}, {-88, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(zeroForceAndTorque2.frame_a, frame_b) annotation(Line(points = {{80, -40}, {90, -40}, {90, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(initAngularVelocity.w_rel_b, derz.u) annotation(Line(points = {{1, 30}, {18, 30}}, color = {0, 0, 127}));
  connect(frame_a, arrow.frame_a) annotation(Line(points = {{-100, 0}, {-88, 0}, {-88, 70}, {-80, 70}}, color = {95, 95, 95}, thickness = 0.5));
  connect(initPosition.r_rel_a, arrow.r_head) annotation(Line(points = {{1, 70}, {10, 70}, {10, 52}, {-70, 52}, {-70, 58}}, color = {0, 0, 127}));
  annotation(Documentation(info = "<html>
<p>
Joint which does not constrain the motion between frame_a and frame_b.
Such a joint is meaningful if the <b>relative</b> distance and orientation
between frame_a and frame_b, and their derivatives, shall be used
as <b>states</b> or shall be used for non-standard
<b>initialization</b>. This joint allows to <b>initialize</b>
every <b>scalar</b> element of the relative quantities, as well
as to define <b>StateSelect</b> attributes for every
<b>scalar</b> element separately.
</p>

<p>
In the following figure the animation of a FreeMotionScalarInit
joint is shown. The light blue coordinate system is
frame_a and the dark blue coordinate system is
frame_b of the joint.
(here: r_rel_a_1(start = 0.5), r_rel_a_2(start = 0), r_rel_a_3(start = 0.5),
       angle_1(start = 45<sup>o</sup>), angle_2(start = 45<sup>o</sup>), angle_3(start = 45<sup>o</sup>)).
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/FreeMotion.png\">
</p>

<p>
A example to use this joint for the initialization of a planar double pendulum by providing
its tip position, is shown in
<a href=\"modelica://Modelica.Mechanics.MultiBody.Examples.Elementary.DoublePendulumInitTip\">Examples.Elementary.DoublePendulumInitTip</a>.
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -84}, {150, -44}}, textString = "%name"), Rectangle(visible = true, origin = {45, 0}, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-135, -5}, {-105, 5}}), Rectangle(visible = true, origin = {4.05, 0}, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-13.05, -5}, {15.95, 5}}), Rectangle(visible = true, origin = {25, 0}, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-75, -5}, {-45, 5}}), Line(visible = true, points = {{-90, 40}, {-60, 74.794}, {-10, 90}, {40, 75.296}, {80, 30}}, color = {128, 128, 128}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30, smooth = Smooth.Bezier), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{90, 0}, {50, 20}, {50, 5}, {30, 5}, {30, -5}, {50, -5}, {50, -20}, {90, 0}})}));
end FreeMotionScalarInit;
