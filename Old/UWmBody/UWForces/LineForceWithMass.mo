within UWmBody.UWForces;

model LineForceWithMass "General line force component with an optional point mass on the connection line"
  import UWmBody.UWTypes;
  extends UWInterfaces.PartialTwoFrames;
  Modelica.Mechanics.Translational.Interfaces.Flange_a flange_b "1-dim. translational flange (connect force of Translational library between flange_a and flange_b)" annotation(Placement(transformation(origin = {60, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true, iconTransformation(origin = {60, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Interfaces.Flange_b flange_a "1-dim. translational flange (connect force of Translational library between flange_a and flange_b)" annotation(Placement(transformation(origin = {-60, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true, iconTransformation(origin = {-60, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  parameter Boolean animateLine = true "= true, if a line shape between frame_a and frame_b shall be visualized";
  parameter Boolean animateMass = true "= true, if point mass shall be visualized as sphere provided m > 0";
  parameter SI.Mass m(min = 0) = 0 "Mass of point mass on the connection line between the origin of frame_a and the origin of frame_b";
  parameter Real lengthFraction(unit = "1", min = 0, max = 1) = 0.5 "Location of point mass with respect to frame_a as a fraction of the distance from frame_a to frame_b";
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(tab = "Animation"));
  parameter UWTypes.ShapeType lineShapeType = "cylinder" "Type of shape visualizing the line from frame_a to frame_b" annotation(Dialog(tab = "Animation", group = "if animateLine = true", enable = animateLine));
  input SI.Length lineShapeWidth = world.defaultArrowDiameter "Width of shape" annotation(Dialog(tab = "Animation", group = "if animateLine = true", enable = animateLine));
  input SI.Length lineShapeHeight = lineShapeWidth "Height of shape" annotation(Dialog(tab = "Animation", group = "if animateLine = true", enable = animateLine));
  parameter UWTypes.ShapeExtra lineShapeExtra = 0.0 "Extra parameter for shape" annotation(Dialog(tab = "Animation", group = "if animateLine = true", enable = animateLine));
  input UWTypes.Color lineShapeColor = UWmBody.UWTypes.Defaults.SensorColor "Color of line shape" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animateLine = true", enable = animateLine));
  input Real massDiameter = world.defaultBodyDiameter "Diameter of point mass sphere" annotation(Dialog(tab = "Animation", group = "if animateMass = true", enable = animateMass));
  input UWTypes.Color massColor = UWmBody.UWTypes.Defaults.BodyColor "Color of point mass" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animateMass = true", enable = animateMass));
  parameter SI.Position s_small = 1.E-10 "Prevent zero-division if distance between frame_a and frame_b is zero" annotation(Dialog(tab = "Advanced"));
  parameter Boolean fixedRotationAtFrame_a = false "=true, if rotation frame_a.R is fixed (to directly connect line forces)" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Advanced", group = "If enabled, can give wrong results, see MultiBody.UsersGuide.Tutorial.ConnectionOfLineForces"));
  parameter Boolean fixedRotationAtFrame_b = false "=true, if rotation frame_b.R is fixed (to directly connect line forces)" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Advanced", group = "If enabled, can give wrong results, see MultiBody.UsersGuide.Tutorial.ConnectionOfLineForces"));
  SI.Distance length "Distance between the origin of frame_a and the origin of frame_b";
  SI.Position r_rel_0[3] "Position vector from frame_a to frame_b resolved in world frame";
  Real e_rel_0[3](each final unit = "1") "Unit vector in direction from frame_a to frame_b, resolved in world frame";
protected
  SI.Force fa "Force from flange_a";
  SI.Force fb "Force from flange_b";
  SI.Position r_CM_0[3](each stateSelect = StateSelect.avoid) "Position vector from world frame to point mass, resolved in world frame";
  Modelica.SIunits.Velocity v_CM_0[3](each stateSelect = StateSelect.avoid) "First derivative of r_CM_0";
  Modelica.SIunits.Acceleration ag_CM_0[3] "der(v_CM_0) - gravityAcceleration";
  UWVisualizers.Advanced.Shape lineShape(shapeType = lineShapeType, color = lineShapeColor, specularCoefficient = specularCoefficient, length = length, width = lineShapeWidth, height = lineShapeHeight, lengthDirection = e_rel_0, widthDirection = Frames.resolve1(frame_a.R, {0, 1, 0}), extra = lineShapeExtra, r = frame_a.r_0) if world.enableAnimation and animateLine;
  UWVisualizers.Advanced.Shape massShape(shapeType = "sphere", color = massColor, specularCoefficient = specularCoefficient, length = massDiameter, width = massDiameter, height = massDiameter, lengthDirection = e_rel_0, widthDirection = {0, 1, 0}, r_shape = e_rel_0 * (length * lengthFraction - massDiameter / 2), r = frame_a.r_0) if world.enableAnimation and animateMass and m > 0;
equation
  assert(noEvent(length > s_small), "
    The distance between the origin of frame_a and the origin of frame_b
    of a LineForceWithMass component became smaller as parameter s_small
    (= a small number, defined in the \"Advanced\" menu). The distance is
    set to s_small, although it is smaller, to avoid a division by zero
    when computing the direction of the line force. Possible reasons
    for this situation:
    - At initial time the distance may already be zero: Change the initial
      positions of the bodies connected by this element.
    - Hardware stops are not modeled or are modeled not stiff enough.
      Include stops, e.g., stiff springs, or increase the stiffness
      if already present.
    - Another error in your model may lead to unrealistically large forces
      and torques that would in reality destroy the stops.
    - The flange_b connector might be defined by a pre-defined motion,
      e.g., with Modelica.Mechanics.Translational.Position and the
      predefined flange_b.s is zero or negative.
    ");
  // Determine relative position vector between the two frames
  r_rel_0 = frame_b.r_0 - frame_a.r_0;
  length = Modelica.Math.Vectors.length(r_rel_0);
  flange_a.s = 0;
  flange_b.s = length;
  e_rel_0 = r_rel_0 / Frames.Internal.maxWithoutEvent(length, s_small);
  // Determine translational flange forces
  if cardinality(flange_a) > 0 and cardinality(flange_b) > 0 then
    fa = flange_a.f;
    fb = flange_b.f;
  elseif cardinality(flange_a) > 0 and cardinality(flange_b) == 0 then
    fa = flange_a.f;
    fb = -fa;
  elseif cardinality(flange_a) == 0 and cardinality(flange_b) > 0 then
    fa = -fb;
    fb = flange_b.f;
  else
    fa = 0;
    fb = 0;
  end if;
  /* Force and torque balance of point mass
         - Kinematics for center of mass CM of point mass including gravity
           r_CM_0 = frame_a.r0 + r_rel_CM_0;
           v_CM_0 = der(r_CM_0);
           ag_CM_0 = der(v_CM_0) - world.gravityAcceleration(r_CM_0);
         - Power balance for the connection line
           (f1=force on frame_a side, f2=force on frame_b side, h=lengthFraction)
           0 = f1*va - m*ag_CM*(va+(vb-va)*h) + f2*vb
             = (f1 - m*ag_CM*(1-h))*va + (f2 - m*ag_CM*h)*vb
           since va and vb are completely independent from other
           the parenthesis must vanish:
             f1 := m*ag_CM*(1-h)
             f2 := m*ag_CM*h
         - Force balance on frame_a and frame_b finally results in
             0 = frame_a.f + e_rel_a*fa - f1_a
             0 = frame_b.f + e_rel_b*fb - f2_b
           and therefore
             frame_a.f = -e_rel_a*fa + m*ag_CM_a*(1-h)
             frame_b.f = -e_rel_b*fb + m*ag_CM_b*h
      */
  if m > 0 then
    r_CM_0 = frame_a.r_0 + r_rel_0 * lengthFraction;
    v_CM_0 = der(r_CM_0);
    ag_CM_0 = der(v_CM_0) - world.gravityAcceleration(r_CM_0);
    frame_a.f = Frames.resolve2(frame_a.R, m * (1 - lengthFraction) * ag_CM_0 - e_rel_0 * fa);
    frame_b.f = Frames.resolve2(frame_b.R, m * lengthFraction * ag_CM_0 - e_rel_0 * fb);
  else
    r_CM_0 = zeros(3);
    v_CM_0 = zeros(3);
    ag_CM_0 = zeros(3);
    frame_a.f = -Frames.resolve2(frame_a.R, e_rel_0 * fa);
    frame_b.f = -Frames.resolve2(frame_b.R, e_rel_0 * fb);
  end if;
  // Provide appropriate equations, if direct connections of line forces
  if fixedRotationAtFrame_a then
    Connections.root(frame_a.R);
    frame_a.R = Frames.nullRotation();
  else
    frame_a.t = zeros(3);
  end if;
  if fixedRotationAtFrame_b then
    Connections.root(frame_b.R);
    frame_b.R = Frames.nullRotation();
  else
    frame_b.t = zeros(3);
  end if;
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {5, 5}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -95}, {150, -55}}, textString = "%name"), Line(visible = true, points = {{-55, 0}, {-55, 23}, {-30, 23}, {-30, 70}, {-60, 70}, {-60, 100}}), Line(visible = true, points = {{55, 0}, {55, 20}, {30, 20}, {30, 70}, {60, 70}, {60, 100}}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-8, -8}, {8, 8}}), Ellipse(visible = fixedRotationAtFrame_a, lineColor = {255, 0, 0}, extent = {{-130, -30}, {-70, 30}}), Text(visible = fixedRotationAtFrame_a, textColor = {255, 0, 0}, extent = {{-140, 30}, {-62, 50}}, textString = "R=0"), Ellipse(visible = fixedRotationAtFrame_b, lineColor = {255, 0, 0}, fillColor = {255, 255, 255}, extent = {{70, -30}, {130, 30}}), Text(visible = fixedRotationAtFrame_b, textColor = {255, 0, 0}, extent = {{62, 30}, {140, 50}}, textString = "R=0"), Line(visible = true, points = {{-47.202, 0}, {54.973, 0}}, color = {64, 64, 64}, pattern = LinePattern.Dot), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{-70, -15}, {-40, 15}}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{40, -15}, {70, 15}}), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-40.019, 25.992}, {-40.019, 25.992}, {-47.214, 28.972}, {-55, 30}, {-62.786, 28.972}, {-69.981, 25.992}, {-76.213, 21.213}, {-80.992, 14.981}, {-83.972, 7.786}, {-85, 0}, {-83.972, -7.786}, {-80.992, -14.981}, {-76.213, -21.213}, {-69.981, -25.992}, {-62.786, -28.972}, {-55, -30}, {-47.214, -28.972}, {-40.019, -25.992}, {-40.019, -25.992}, {-39.693, -36.955}, {-39.693, -36.955}, {-47.16, -39.224}, {-55, -40}, {-62.84, -39.224}, {-70.307, -36.955}, {-77.192, -33.279}, {-83.284, -28.284}, {-88.279, -22.192}, {-91.955, -15.307}, {-94.224, -7.84}, {-95, 0}, {-94.224, 7.84}, {-91.955, 15.307}, {-88.279, 22.192}, {-83.284, 28.284}, {-77.192, 33.279}, {-70.307, 36.955}, {-62.84, 39.224}, {-55, 40}, {-47.16, 39.224}, {-39.693, 36.955}, {-39.693, 36.955}}, smooth = Smooth.Bezier), Polygon(visible = true, rotation = 180, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-40.019, 25.992}, {-40.019, 25.992}, {-47.214, 28.972}, {-55, 30}, {-62.786, 28.972}, {-69.981, 25.992}, {-76.213, 21.213}, {-80.992, 14.981}, {-83.972, 7.786}, {-85, 0}, {-83.972, -7.786}, {-80.992, -14.981}, {-76.213, -21.213}, {-69.981, -25.992}, {-62.786, -28.972}, {-55, -30}, {-47.214, -28.972}, {-40.019, -25.992}, {-40.019, -25.992}, {-39.693, -36.955}, {-39.693, -36.955}, {-47.16, -39.224}, {-55, -40}, {-62.84, -39.224}, {-70.307, -36.955}, {-77.192, -33.279}, {-83.284, -28.284}, {-88.279, -22.192}, {-91.955, -15.307}, {-94.224, -7.84}, {-95, 0}, {-94.224, 7.84}, {-91.955, 15.307}, {-88.279, 22.192}, {-83.284, 28.284}, {-77.192, 33.279}, {-70.307, 36.955}, {-62.84, 39.224}, {-55, 40}, {-47.16, 39.224}, {-39.693, 36.955}, {-39.693, 36.955}}, smooth = Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{-60, 80}, {60, 80}}, color = {64, 64, 64}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 15), Text(visible = true, origin = {8.78, 4.5}, textColor = {64, 64, 64}, extent = {{-48.78, 78}, {31.22, 93}}, textString = "length"), Line(visible = true, points = {{-60, 0}, {-60, 24}, {-40, 24}, {-40, 60}, {-60, 60}, {-60, 100}}, color = {64, 64, 64}), Line(visible = true, points = {{60, 1}, {60, 21}, {40, 21}, {40, 60}, {60, 60}, {60, 100}}, color = {64, 64, 64}), Line(visible = true, points = {{-60, 0}, {60, 0}}, color = {64, 64, 64}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, extent = {{-8, -8}, {8, 8}}), Line(visible = true, points = {{-60, 0}, {-31, 0}}, color = {64, 64, 64}), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-19, 0}, {-31, 3}, {-31, -3}, {-19, 0}}), Line(visible = true, points = {{-60, 16}, {0, 16}}, color = {64, 64, 64}), Line(visible = true, points = {{0, 0}, {0, 20}}, color = {64, 64, 64}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-49, -21}, {-8, -11}}, textString = "e_rel_0"), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{0, 16}, {-12, 19}, {-12, 13}, {0, 16}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-50, 26}, {51, 35}}, textString = "length*lengthFraction"), Line(visible = true, points = {{-17, 26}, {-26, 16}}, color = {64, 64, 64}, pattern = LinePattern.Dot), Line(visible = true, points = {{-31, -13}, {-40, 0}}, color = {64, 64, 64}, pattern = LinePattern.Dot), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{45, -15}, {75, 15}}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{-75.846, -15}, {-45.846, 15}}), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-45.019, 25.992}, {-45.019, 25.992}, {-52.214, 28.972}, {-60, 30}, {-67.786, 28.972}, {-74.981, 25.992}, {-81.213, 21.213}, {-85.992, 14.981}, {-88.972, 7.786}, {-90, 0}, {-88.972, -7.786}, {-85.992, -14.981}, {-81.213, -21.213}, {-74.981, -25.992}, {-67.786, -28.972}, {-60, -30}, {-52.214, -28.972}, {-45.019, -25.992}, {-45.019, -25.992}, {-44.693, -36.955}, {-44.693, -36.955}, {-52.16, -39.224}, {-60, -40}, {-67.84, -39.224}, {-75.307, -36.955}, {-82.192, -33.279}, {-88.284, -28.284}, {-93.279, -22.192}, {-96.955, -15.307}, {-99.224, -7.84}, {-100, 0}, {-99.224, 7.84}, {-96.955, 15.307}, {-93.279, 22.192}, {-88.284, 28.284}, {-82.192, 33.279}, {-75.307, 36.955}, {-67.84, 39.224}, {-60, 40}, {-52.16, 39.224}, {-44.693, 36.955}, {-44.693, 36.955}}, smooth = Smooth.Bezier), Polygon(visible = true, rotation = 180, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-45.019, 25.992}, {-45.019, 25.992}, {-52.214, 28.972}, {-60, 30}, {-67.786, 28.972}, {-74.981, 25.992}, {-81.213, 21.213}, {-85.992, 14.981}, {-88.972, 7.786}, {-90, 0}, {-88.972, -7.786}, {-85.992, -14.981}, {-81.213, -21.213}, {-74.981, -25.992}, {-67.786, -28.972}, {-60, -30}, {-52.214, -28.972}, {-45.019, -25.992}, {-45.019, -25.992}, {-44.693, -36.955}, {-44.693, -36.955}, {-52.16, -39.224}, {-60, -40}, {-67.84, -39.224}, {-75.307, -36.955}, {-82.192, -33.279}, {-88.284, -28.284}, {-93.279, -22.192}, {-96.955, -15.307}, {-99.224, -7.84}, {-100, 0}, {-99.224, 7.84}, {-96.955, 15.307}, {-93.279, 22.192}, {-88.284, 28.284}, {-82.192, 33.279}, {-75.307, 36.955}, {-67.84, 39.224}, {-60, 40}, {-52.16, 39.224}, {-44.693, 36.955}, {-44.693, 36.955}}, smooth = Smooth.Bezier)}), Documentation(info = "<html>
<p>
This component is used to exert a <b>line force</b>
between the origin of frame_a and the origin of frame_b
by attaching components of the <b>1-dimensional translational</b>
mechanical library of Modelica (Modelica.Mechanics.Translational)
between the two flange connectors <b>flange_a</b> and
<b>flange_b</b>. Optionally, there is a <b>point mass</b> on the line
connecting the origin of frame_a and the origin of frame_b.
This point mass approximates the <b>mass</b> of the <b>force element</b>.
The distance of the point mass from frame_a as a fraction of the
distance between frame_a and frame_b is defined via
parameter <b>lengthFraction</b> (default is 0.5, i.e., the point
mass is in the middle of the line).
</p>
<p>
In the translational library there is the implicit assumption that
forces of components that have only one flange connector act with
opposite sign on the bearings of the component. This assumption
is also used in the LineForceWithMass component: If a connection
is present to only one of the flange connectors, then the force
in this flange connector acts implicitly with opposite sign also
in the other flange connector.
</p>
</html>"));
end LineForceWithMass;
