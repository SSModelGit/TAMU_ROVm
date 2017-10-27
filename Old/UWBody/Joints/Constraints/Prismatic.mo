within UWBody.Joints.Constraints;

model Prismatic "Prismatic cut-joint and translational directions may be constrained or released"
  extends UWBody.Interfaces.PartialTwoFrames;
  import Cv = Modelica.SIunits.Conversions;
  parameter Boolean x_locked = true "= true: constraint force in x-direction, resolved in frame_a" annotation(Dialog(group = "Constraints"), choices(checkBox = true));
  parameter Boolean y_locked = true "= true: constraint force in y-direction, resolved in frame_a" annotation(Dialog(group = "Constraints"), choices(checkBox = true));
  parameter Boolean z_locked = true "= true: constraint force in z-direction, resolved in frame_a" annotation(Dialog(group = "Constraints"), choices(checkBox = true));
  parameter Boolean animation = true "= true, if animation shall be enabled (show sphere)";
  parameter SI.Distance sphereDiameter = world.defaultJointLength / 3 "Diameter of sphere representing the spherical joint" annotation(Dialog(group = "if animation = true", enable = animation));
  input Types.Color sphereColor = Types.Defaults.JointColor "Color of sphere representing the spherical joint" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "if animation = true", enable = animation));
protected
  Frames.Orientation R_rel "Dummy or relative orientation object from frame_a to frame_b";
  Modelica.SIunits.Position r_rel_a[3] "Position vector from origin of frame_a to origin of frame_b, resolved in frame_a";
  Modelica.SIunits.InstantaneousPower P;
public
  Visualizers.Advanced.Shape sphere(shapeType = "sphere", color = sphereColor, specularCoefficient = specularCoefficient, length = sphereDiameter, width = sphereDiameter, height = sphereDiameter, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, r_shape = {-0.5, 0, 0} * sphereDiameter, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
equation
  // Determine relative position vector resolved in frame_a
  R_rel = Frames.relativeRotation(frame_a.R, frame_b.R);
  r_rel_a = Frames.resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
  // Constraint equations concerning rotations
  ones(3) = {R_rel.T[1, 1], R_rel.T[2, 2], R_rel.T[3, 3]};
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
  zeros(3) = frame_a.t + Frames.resolve1(R_rel, frame_b.t) + cross(r_rel_a, Frames.resolve1(R_rel, frame_b.f));
  zeros(3) = Frames.resolve1(R_rel, frame_b.f) + frame_a.f;
  P = frame_a.t * Frames.angularVelocity2(frame_a.R) + frame_b.t * Frames.angularVelocity2(frame_b.R) + frame_b.f * Frames.resolve2(frame_b.R, der(frame_b.r_0)) + frame_a.f * Frames.resolve2(frame_a.R, der(frame_a.r_0));
  annotation(defaultComponentName = "constraint", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = x_locked and not y_locked and not z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: x"), Text(visible = not x_locked and y_locked and not z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: y"), Text(visible = not x_locked and not y_locked and z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: z"), Text(visible = x_locked and y_locked and not z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: x, y"), Text(visible = x_locked and not y_locked and z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: x, z"), Text(visible = not x_locked and y_locked and z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: y, z"), Text(visible = x_locked and y_locked and z_locked, textColor = {95, 95, 95}, extent = {{-100, -100}, {100, -70}}, textString = "lock: x, y, z"), Line(visible = true, points = {{100, -26}, {100, 25}}), Text(visible = true, origin = {0, 10}, textColor = {64, 64, 64}, extent = {{-150, 80}, {150, 120}}, textString = "%name"), Polygon(visible = true, origin = {-32.5, 5}, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-7.5, 45}, {7.5, 52.5}, {7.5, -42.5}, {-7.5, -55}}), Rectangle(visible = true, origin = {-14.286, -1.648}, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-85.714, -48.352}, {-25.714, 51.648}}), Polygon(visible = true, origin = {-65, 55}, fillColor = {217, 217, 217}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-35, -5}, {-35, -5}, {-35, 2.5}, {40, 2.5}, {25, -5}}), Line(visible = true, origin = {-40, 0}, points = {{15, 57.5}, {0, 50}, {-60, 50}, {0, 50}, {0, -50}}, color = {255, 255, 255}), Polygon(visible = true, origin = {-55.276, 7.5}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, points = {{-44.724, 42.5}, {-44.724, 50}, {30.276, 50}, {30.276, -45}, {15.276, -57.5}, {-44.724, -57.5}}), Rectangle(visible = true, origin = {-3.846, 1.2}, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-31.154, -31.2}, {103.846, 28.8}}), Polygon(visible = true, origin = {34.375, 32.5}, lineColor = {64, 64, 64}, fillColor = {217, 217, 217}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, points = {{-69.375, -2.5}, {-61.875, 2.5}, {65.625, 2.5}, {65.625, -2.5}}), Line(visible = true, origin = {32.5, 30}, points = {{-67.5, 0}, {67.5, 0}}, color = {255, 255, 255}), Polygon(visible = true, origin = {20.5, 8}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, points = {{-55.5, 22}, {-48, 27}, {79.5, 27}, {79.5, -38}, {-55.5, -38}}), Line(visible = true, origin = {30, 0}, points = {{-40, 0}, {40, 0}}, color = {64, 64, 64}, arrow = {Arrow.Filled, Arrow.Filled}, arrowSize = 20), Rectangle(visible = true, origin = {30, 61.716}, rotation = 45, lineColor = {190, 20, 31}, fillColor = {190, 20, 31}, fillPattern = FillPattern.Solid, extent = {{-20, -20}, {20, 20}}), Line(visible = true, origin = {30, 61.716}, points = {{-20, -5}, {-7.5, 7.5}, {7.5, -7.5}, {20, 5}}, color = {255, 255, 255}, thickness = 5)}), Documentation(info = "<html>
<p>This model does not use explicit variables e.g. state variables in order to describe the relative motion of frame_b with respect to frame_a, but defines kinematic constraints between the frame_a and frame_b. The forces and torques at both frames are then evaluated in such a way that the constraints are satisfied.  Sometimes this type of formulation is also called an implicit joint in literature.</p>
<p>As a consequence of the formulation the relative kinematics between frame_a and frame_b cannot be initialized.</p>
<p>In particular in complex multibody systems with closed loops this may help to simplify the system of non-linear equations. Please compare the translation log using the classical joint formulation and the alternative formulation used here in order to check whether this fact applies to the particular system under consideration.</p>
<p>In systems without closed loops the use of this implicit joint does not make sense or may even be disadvantageous.</p>
<p>See the subpackage <a href=\"Modelica://UWBody.Examples.Constraints\">Examples.Constraints</a> for testing the joint. </p>
</html>"));
end Prismatic;
