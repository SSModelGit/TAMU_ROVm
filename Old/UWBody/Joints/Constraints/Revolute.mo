within UWBody.Joints.Constraints;

model Revolute "Revolute cut-joint and translational directions may be constrained or released"
  extends UWBody.Interfaces.PartialTwoFrames;
  parameter Boolean x_locked = true "= true: constraint force in x-direction, resolved in frame_a" annotation(Dialog(group = "Constraints in translational motion"), choices(checkBox = true));
  parameter Boolean y_locked = true "= true: constraint force in y-direction, resolved in frame_a" annotation(Dialog(group = "Constraints in translational motion"), choices(checkBox = true));
  parameter Boolean z_locked = true "= true: constraint force in z-direction, resolved in frame_a" annotation(Dialog(group = "Constraints in translational motion"), choices(checkBox = true));
  parameter Boolean animation = true "= true, if animation shall be enabled (show sphere)";
  parameter Types.Axis n = {0, 1, 0} "Axis of rotation resolved in frame_a (= same as in frame_b)" annotation(Evaluate = true);
  parameter SI.Distance sphereDiameter = world.defaultJointLength / 3 "Diameter of sphere representing the spherical joint" annotation(Dialog(group = "if animation = true", enable = animation));
  input Types.Color sphereColor = Types.Defaults.JointColor "Color of sphere representing the spherical joint" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "if animation = true", enable = animation));
protected
  Frames.Orientation R_rel "Dummy or relative orientation object from frame_a to frame_b";
  Modelica.SIunits.Position r_rel_a[3] "Position vector from origin of frame_a to origin of frame_b, resolved in frame_a";
  Modelica.SIunits.InstantaneousPower P;
  parameter Real e[3](each final unit = "1") = Modelica.Math.Vectors.normalizeWithAssert(n) "Unit vector in direction of rotation axis, resolved in frame_a (= same as in frame_b)";
  parameter Real nnx_a[3](each final unit = "1") = if abs(e[1]) > 0.1 then {0, 1, 0} else if abs(e[2]) > 0.1 then {0, 0, 1} else {1, 0, 0} "Arbitrary vector that is not aligned with rotation axis n" annotation(Evaluate = true);
  parameter Real ey_a[3](each final unit = "1") = Modelica.Math.Vectors.normalizeWithAssert(cross(e, nnx_a)) "Unit vector orthogonal to axis n of revolute joint, resolved in frame_a" annotation(Evaluate = true);
  parameter Real ex_a[3](each final unit = "1") = cross(ey_a, e) "Unit vector orthogonal to axis n of revolute joint and to ey_a, resolved in frame_a";
public
  Visualizers.Advanced.Shape sphere(shapeType = "sphere", color = sphereColor, specularCoefficient = specularCoefficient, length = sphereDiameter, width = sphereDiameter, height = sphereDiameter, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, r_shape = {-0.5, 0, 0} * sphereDiameter, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
equation
  // Determine relative position vector resolved in frame_a
  R_rel = Frames.relativeRotation(frame_a.R, frame_b.R);
  r_rel_a = Frames.resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
  // Constraint equations concerning translations
  if x_locked and y_locked and z_locked then
    r_rel_a = zeros(3);
  elseif x_locked and y_locked and not z_locked then
    r_rel_a[1] = 0;
    r_rel_a[2] = 0;
    frame_a.f[3] = 0;
  elseif x_locked and not y_locked and z_locked then
    r_rel_a[1] = 0;
    r_rel_a[3] = 0;
    frame_a.f[2] = 0;
  elseif x_locked and not y_locked and not z_locked then
    r_rel_a[1] = 0;
    frame_a.f[2] = 0;
    frame_a.f[3] = 0;
  elseif not x_locked and y_locked and z_locked then
    r_rel_a[2] = 0;
    r_rel_a[3] = 0;
    frame_a.f[1] = 0;
  elseif not x_locked and y_locked and not z_locked then
    r_rel_a[2] = 0;
    frame_a.f[1] = 0;
    frame_a.f[3] = 0;
  elseif not x_locked and not y_locked and z_locked then
    r_rel_a[3] = 0;
    frame_a.f[1] = 0;
    frame_a.f[2] = 0;
  else
    frame_a.f = zeros(3);
  end if;
  // Constraint equations concerning rotations
  0 = ex_a * R_rel.T * e;
  0 = ey_a * R_rel.T * e;
  frame_a.t * n = 0;
  zeros(3) = frame_a.f + Frames.resolve1(R_rel, frame_b.f);
  zeros(3) = frame_a.t + Frames.resolve1(R_rel, frame_b.t) - cross(r_rel_a, frame_a.f);
  P = frame_a.t * Frames.angularVelocity2(frame_a.R) + frame_b.t * Frames.angularVelocity2(frame_b.R) + Frames.resolve1(frame_b.R, frame_b.f) * der(frame_b.r_0) + Frames.resolve1(frame_a.R, frame_a.f) * der(frame_a.r_0);
  annotation(defaultComponentName = "constraint", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-63, -93}, {53, -63}}, textString = "n=%n"), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, -60}, {-30, 60}}, radius = 10), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{30, -60}, {100, 60}}, radius = 10), Rectangle(visible = true, lineColor = {64, 64, 64}, extent = {{-100, -60}, {-30, 60}}, radius = 10), Rectangle(visible = true, lineColor = {64, 64, 64}, extent = {{30, -60}, {100, 60}}, radius = 10), Text(visible = true, textColor = {64, 64, 64}, extent = {{-90, -11}, {-54, 14}}, textString = "a"), Text(visible = true, textColor = {64, 64, 64}, extent = {{51, -14}, {87, 11}}, textString = "b"), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-30, -10}, {30, 11}}), Text(visible = true, origin = {0, 3}, textColor = {64, 64, 64}, extent = {{-47, 87}, {47, 110}}, textString = "constraint"), Text(visible = true, origin = {0, 3}, textColor = {64, 64, 64}, extent = {{-150, 110}, {150, 150}}, textString = "%name"), Rectangle(visible = true, origin = {-0, 60}, rotation = 45, lineColor = {190, 20, 31}, fillColor = {190, 20, 31}, fillPattern = FillPattern.Solid, extent = {{-20, -20}, {20, 20}}), Line(visible = true, origin = {-0, 60}, points = {{-20, -5}, {-7.5, 7.5}, {7.5, -7.5}, {20, 5}}, color = {255, 255, 255}, thickness = 5)}), Documentation(info = "<html>
<p>This model does not use explicit variables e.g. state variables in order to describe the relative motion of frame_b with respect to frame_a, but defines kinematic constraints between the frame_a and frame_b. The forces and torques at both frames are then evaluated in such a way that the constraints are satisfied. Sometimes this type of formulation is also called an implicit joint in literature.</p>
<p>As a consequence of the formulation the relative kinematics between frame_a and frame_b cannot be initialized.</p>
<p>In particular in complex multibody systems with closed loops this may help to simplify the system of non-linear equations. Please compare the translation log using the classical joint formulation and the alternative formulation used here in order to check whether this fact applies to the particular system under consideration.</p>
<p>In systems without closed loops the use of this implicit joint does not make sense or may even be disadvantageous.</p>
<p>See the subpackage <a href=\"Modelica://UWBody.Examples.Constraints\">Examples.Constraints</a> for testing the joint. </p>
</html>"));
end Revolute;
