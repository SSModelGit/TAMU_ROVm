within UWBody.Joints.Constraints;

model Spherical "Spherical cut joint and translational directions may be constrained or released"
  extends UWBody.Interfaces.PartialTwoFrames;
  import MBS = UWBody;
  parameter Boolean x_locked = true "= true: constraint force in x-direction, resolved in frame_a" annotation(Dialog(group = "Constraints"), choices(checkBox = true));
  parameter Boolean y_locked = true "= true: constraint force in y-direction, resolved in frame_a" annotation(Dialog(group = "Constraints"), choices(checkBox = true));
  parameter Boolean z_locked = true "= true: constraint force in z-direction, resolved in frame_a" annotation(Dialog(group = "Constraints"), choices(checkBox = true));
  parameter Boolean animation = true "= true, if animation shall be enabled (show sphere)" annotation(Dialog(group = "Animation"));
  parameter Modelica.SIunits.Distance sphereDiameter = world.defaultJointLength / 3 "Diameter of sphere representing the spherical joint" annotation(Dialog(group = "Animation", enable = animation));
  input MBS.Types.Color sphereColor = MBS.Types.Defaults.JointColor "Color of sphere representing the spherical joint" annotation(Dialog(colorSelector = true, group = "Animation", enable = animation));
  input MBS.Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "Animation", enable = animation));
  UWBody.Visualizers.Advanced.Shape sphere(shapeType = "sphere", color = sphereColor, specularCoefficient = specularCoefficient, length = sphereDiameter, width = sphereDiameter, height = sphereDiameter, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, r_shape = {-0.5, 0, 0} * sphereDiameter, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
protected
  MBS.Frames.Orientation R_rel "Dummy or relative orientation object from frame_a to frame_b";
  Modelica.SIunits.Position r_rel_a[3] "Position vector from origin of frame_a to origin of frame_b, resolved in frame_a";
  Modelica.SIunits.InstantaneousPower P;
equation
  // Determine relative position vector resolved in frame_a
  R_rel = MBS.Frames.relativeRotation(frame_a.R, frame_b.R);
  r_rel_a = MBS.Frames.resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
  // Constraint equations concerning translation
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
  //frame_a.t = zeros(3);
  frame_b.t = zeros(3);
  frame_b.f = -MBS.Frames.resolve2(R_rel, frame_a.f);
  zeros(3) = frame_a.t + MBS.Frames.resolve1(R_rel, frame_b.t) - cross(r_rel_a, frame_a.f);
  P = frame_a.t * MBS.Frames.angularVelocity2(frame_a.R) + frame_b.t * MBS.Frames.angularVelocity2(frame_b.R) + MBS.Frames.resolve1(frame_b.R, frame_b.f) * der(frame_b.r_0) + MBS.Frames.resolve1(frame_a.R, frame_a.f) * der(frame_a.r_0);
  annotation(defaultComponentName = "constraint", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 80}, {150, 120}}, textString = "%name"), Text(visible = x_locked and not y_locked and not z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: x"), Text(visible = not x_locked and y_locked and not z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: y"), Text(visible = not x_locked and not y_locked and z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: z"), Text(visible = x_locked and y_locked and not z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: x, y"), Text(visible = x_locked and not y_locked and z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: x, z"), Text(visible = not x_locked and y_locked and z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: y, z"), Text(visible = x_locked and y_locked and z_locked, textColor = {64, 64, 64}, extent = {{-100, -100}, {100, -70}}, textString = "lock: x, y, z"), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, -10}, {-68, 10}}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.HorizontalCylinder, extent = {{23, -10}, {100, 10}}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Sphere, extent = {{-24, -25}, {26, 25}}), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{29.35, 40.479}, {29.35, 40.479}, {22.678, 44.561}, {15.473, 47.545}, {7.869, 49.377}, {0, 50}, {-7.869, 49.377}, {-15.473, 47.545}, {-22.678, 44.561}, {-29.35, 40.479}, {-35.355, 35.355}, {-40.479, 29.35}, {-44.561, 22.678}, {-47.545, 15.473}, {-49.377, 7.869}, {-50, 0}, {-49.377, -7.869}, {-47.545, -15.473}, {-44.561, -22.678}, {-40.479, -29.35}, {-35.355, -35.355}, {-29.35, -40.479}, {-22.678, -44.561}, {-15.473, -47.545}, {-7.869, -49.377}, {0, -50}, {7.869, -49.377}, {15.473, -47.545}, {22.678, -44.561}, {29.35, -40.479}, {29.35, -40.479}, {29.989, -62.327}, {29.989, -62.327}, {22.866, -65.277}, {15.449, -67.419}, {7.805, -68.725}, {0, -69.167}, {-7.805, -68.725}, {-15.449, -67.419}, {-22.866, -65.277}, {-29.989, -62.327}, {-36.748, -58.597}, {-43.077, -54.114}, {-48.908, -48.908}, {-54.114, -43.077}, {-58.597, -36.748}, {-62.327, -29.989}, {-65.277, -22.866}, {-67.419, -15.449}, {-68.725, -7.805}, {-69.167, 0}, {-68.725, 7.805}, {-67.419, 15.449}, {-65.277, 22.866}, {-62.327, 29.989}, {-58.597, 36.748}, {-54.114, 43.077}, {-48.908, 48.908}, {-43.077, 54.114}, {-36.748, 58.597}, {-29.989, 62.327}, {-22.866, 65.277}, {-15.449, 67.419}, {-7.805, 68.725}, {0, 69.167}, {7.805, 68.725}, {15.449, 67.419}, {22.866, 65.277}, {29.989, 62.327}, {29.989, 62.327}}, smooth = Smooth.Bezier), Rectangle(visible = true, origin = {68.284, 51.716}, rotation = 45, lineColor = {190, 20, 31}, fillColor = {190, 20, 31}, fillPattern = FillPattern.Solid, extent = {{-20, -20}, {20, 20}}), Line(visible = true, origin = {68.284, 51.716}, points = {{-20, -5}, {-7.5, 7.5}, {7.5, -7.5}, {20, 5}}, color = {255, 255, 255}, thickness = 5)}), Documentation(info = "<html>
<p>This model does not use explicit variables e.g. state variables in order to describe the relative motion of frame_b with to respect to frame_a, but defines kinematic constraints between the frame_a and frame_b. The forces and torques at both frames are then evaluated in such a way that the constraints are satisfied. Sometimes this type of formulation is also called an implicit joint in literature.</p>
<p>As a consequence of the formulation the relative kinematics between frame_a and frame_b cannot be initialized.</p>
<p>In particular in complex multibody systems with closed loops this may help to simplify the system of non-linear equations. Please compare the translation log using the classical joint formulation and the alternative formulation used here in order to check whether this fact applies to the particular system under consideration.</p>
<p>In systems without closed loops the use of this implicit joint does not make sense or may even be disadvantageous.</p>
<p>See the subpackage <a href=\"Modelica://UWBody.Examples.Constraints\">Examples.Constraints</a> for testing the joint. </p>
</html>"));
end Spherical;
