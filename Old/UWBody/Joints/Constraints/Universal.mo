within UWBody.Joints.Constraints;

model Universal "Universal cut-joint and translational directions may be constrained or released"
  extends UWBody.Interfaces.PartialTwoFrames;
  import MBS = UWBody;
  parameter MBS.Types.Axis n_a = {1, 0, 0} "Axis of revolute joint 1 resolved in frame_a" annotation(Evaluate = true);
  parameter MBS.Types.Axis n_b = {0, 1, 0} "Axis of revolute joint 2 resolved in frame_b" annotation(Evaluate = true);
  parameter Boolean x_locked = true "= true: constraint force in x-direction, resolved in frame_a" annotation(Dialog(group = "Constraints in translational motion"), choices(checkBox = true));
  parameter Boolean y_locked = true "= true: constraint force in y-direction, resolved in frame_a" annotation(Dialog(group = "Constraints in translational motion"), choices(checkBox = true));
  parameter Boolean z_locked = true "= true: constraint force in z-direction, resolved in frame_a" annotation(Dialog(group = "Constraints in translational motion"), choices(checkBox = true));
  parameter Boolean animation = true "= true, if animation shall be enabled (show sphere)" annotation(Dialog(group = "Animation"));
  parameter SI.Distance sphereDiameter = world.defaultJointLength / 3 "Diameter of sphere representing the spherical joint" annotation(Dialog(group = "Animation", enable = animation));
  input MBS.Types.Color sphereColor = MBS.Types.Defaults.JointColor "Color of sphere representing the spherical joint" annotation(Dialog(colorSelector = true, group = "Animation", enable = animation));
  input MBS.Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "Animation", enable = animation));
protected
  MBS.Frames.Orientation R_rel "Dummy or relative orientation object from frame_a to frame_b";
  Real w_rel[3];
  Modelica.SIunits.Position r_rel_a[3] "Position vector from origin of frame_a to origin of frame_b, resolved in frame_a";
  Modelica.SIunits.InstantaneousPower P;
equation
  // Determine relative position vector resolved in frame_a
  R_rel = MBS.Frames.relativeRotation(frame_a.R, frame_b.R);
  w_rel = MBS.Frames.angularVelocity1(R_rel);
  r_rel_a = MBS.Frames.resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
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
  frame_a.t * n_a = 0;
  frame_b.t * n_b = 0;
  n_b * R_rel.T * n_a = 0;
  assert(abs(n_a * n_b) < Modelica.Constants.eps, "The two axes that constitute the Constraints.Universal joint must be different");
  zeros(3) = frame_a.f + MBS.Frames.resolve1(R_rel, frame_b.f);
  zeros(3) = frame_a.t + MBS.Frames.resolve1(R_rel, frame_b.t) - cross(r_rel_a, frame_a.f);
  P = frame_a.t * MBS.Frames.angularVelocity2(frame_a.R) + frame_b.t * MBS.Frames.angularVelocity2(frame_b.R) + MBS.Frames.resolve1(frame_b.R, frame_b.f) * der(frame_b.r_0) + MBS.Frames.resolve1(frame_a.R, frame_a.f) * der(frame_a.r_0);
  annotation(defaultComponentName = "constraint", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = x_locked and not y_locked and not z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: x"), Text(visible = not x_locked and y_locked and not z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: y"), Text(visible = not x_locked and not y_locked and z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: z"), Text(visible = x_locked and y_locked and not z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: x, y"), Text(visible = x_locked and not y_locked and z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: x, z"), Text(visible = not x_locked and y_locked and z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: y, z"), Text(visible = x_locked and y_locked and z_locked, textColor = {64, 64, 64}, extent = {{-100, -106}, {100, -76}}, textString = "lock: x, y, z"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 80}, {150, 120}}, textString = "%name"), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-30, 35}, {-35, 30}, {-35, -30}, {-30, -35}, {30, -35}, {35, -30}, {35, 30}, {30, 35}}), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{25, 65}, {-45, 65}, {-65, 45}, {-65, 20}, {-70, 15}, {-100, 15}, {-100, -15}, {-70, -15}, {-65, -20}, {-65, -45}, {-45, -65}, {25, -65}, {25, -45}, {-35, -45}, {-45, -35}, {-45, 35}, {-35, 45}, {25, 45}}), Polygon(visible = true, origin = {51.25, 0}, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{-76.25, 25}, {8.75, 25}, {18.75, 15}, {48.75, 15}, {48.75, -15}, {18.75, -15}, {8.75, -25}, {-76.25, -25}}), Ellipse(visible = true, rotation = 45, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-10, -10}, {10, 10}}), Rectangle(visible = true, origin = {0, -40}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.VerticalCylinder, extent = {{-10, -5}, {10, 5}}), Rectangle(visible = true, origin = {-85, 0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-15, -15}, {15, 15}}), Rectangle(visible = true, origin = {0, 40}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.VerticalCylinder, extent = {{-10, -5}, {10, 5}}), Rectangle(visible = true, origin = {85, 0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-15, -15}, {15, 15}}), Rectangle(visible = true, origin = {70, 51.716}, rotation = 45, lineColor = {190, 20, 31}, fillColor = {190, 20, 31}, fillPattern = FillPattern.Solid, extent = {{-20, -20}, {20, 20}}), Line(visible = true, origin = {70, 51.716}, points = {{-20, -5}, {-7.5, 7.5}, {7.5, -7.5}, {20, 5}}, color = {255, 255, 255}, thickness = 5)}), Documentation(info = "<html>
<p>This model does not use explicit variables e.g. state variables in order to describe the relative motion of frame_b with respect to frame_a, but defines kinematic constraints between the frame_a and frame_b. The forces and torques at both frames are then evaluated in such a way that the constraints are satisfied. Sometimes this type of formulation is also called an implicit joint in literature.</p>
<p>As a consequence of the formulation the relative kinematics between frame_a and frame_b cannot be initialized.</p>
<p>In particular in complex multibody systems with closed loops this may help to simplify the system of non-linear equations. Please compare the translation log using the classical joint formulation and the alternative formulation used here in order to check whether this fact applies to the particular system under consideration.</p>
<p>In systems without closed loops the use of this implicit joint does not make sense or may even be disadvantageous.</p>
<p>See the subpackage <a href=\"Modelica://UWBody.Examples.Constraints\">Examples.Constraints</a> for testing the joint. </p>
</html>"));
end Universal;
