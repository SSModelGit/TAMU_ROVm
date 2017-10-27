within UWBody.Forces;

model LineForceWithTwoMasses "General line force component with two optional point masses on the connection line"
  import UWBody.Types;
  extends Interfaces.PartialTwoFrames;
  Modelica.Mechanics.Translational.Interfaces.Flange_a flange_b "1-dim. translational flange (connect force of Translational library between flange_a and flange_b)" annotation(Placement(transformation(origin = {60, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true, iconTransformation(origin = {60, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Interfaces.Flange_b flange_a "1-dim. translational flange (connect force of Translational library between flange_a and flange_b)" annotation(Placement(transformation(origin = {-60, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true, iconTransformation(origin = {-60, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  parameter Boolean animate = true "= true, if animation shall be enabled";
  parameter Boolean animateMasses = true "= true, if point masses shall be visualized provided animate=true and m_a, m_b > 0";
  parameter SI.Mass m_a(min = 0) = 0 "Mass of point mass a on the connection line between the origin of frame_a and the origin of frame_b";
  parameter SI.Mass m_b(min = 0) = 0 "Mass of point mass b on the connection line between the origin of frame_a and the origin of frame_b";
  parameter SI.Position L_a = 0 "Distance between point mass a and frame_a (positive, if in direction of frame_b)";
  parameter SI.Position L_b = L_a "Distance between point mass b and frame_b (positive, if in direction of frame_a)";
  input SI.Diameter cylinderDiameter_a = world.defaultForceWidth "Diameter of cylinder at frame_a" annotation(Dialog(tab = "Animation", group = "Cylinder at frame_a if animation = true", enable = animate));
  parameter SI.Length cylinderLength_a = 2 * L_a "Length of cylinder at frame_a" annotation(Dialog(tab = "Animation", group = "Cylinder at frame_a if animation = true", enable = animate));
  input Types.Color color_a = {155, 155, 155} "Color of cylinder at frame_a" annotation(Dialog(colorSelector = true, tab = "Animation", group = "Cylinder at frame_a if animation = true", enable = animate));
  input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(tab = "Animation", group = "Cylinder at frame_a if animation = true", enable = animate));
  input Real diameterFraction = 0.8 "Diameter of cylinder at frame_b with respect to diameter of cylinder at frame_a" annotation(Dialog(tab = "Animation", group = "Cylinder at frame_b if animation = true", enable = animate));
  parameter SI.Length cylinderLength_b = 2 * L_b "Length of cylinder at frame_b" annotation(Dialog(tab = "Animation", group = "Cylinder at frame_b if animation = true", enable = animate));
  input Types.Color color_b = {100, 100, 100} "Color of cylinder at frame_b" annotation(Dialog(colorSelector = true, tab = "Animation", group = "Cylinder at frame_b if animation = true", enable = animate));
  input Real massDiameterFaction = 1.7 "Diameter of point mass spheres with respect to cylinderDiameter_a" annotation(Dialog(tab = "Animation", group = "if animation = true and animateMasses = true", enable = animate and animateMasses));
  input Types.Color massColor = UWBody.Types.Defaults.BodyColor "Color of point masses" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true and animateMasses = true", enable = animate and animateMasses));
  parameter SI.Position s_small = 1.E-10 "Prevent zero-division if distance between frame_a and frame_b is zero" annotation(Dialog(tab = "Advanced"));
  parameter Boolean fixedRotationAtFrame_a = false "=true, if rotation frame_a.R is fixed (to directly connect line forces)" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Advanced", group = "If enabled, can give wrong results, see MultiBody.UsersGuide.Tutorial.ConnectionOfLineForces"));
  parameter Boolean fixedRotationAtFrame_b = false "=true, if rotation frame_b.R is fixed (to directly connect line forces)" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Advanced", group = "If enabled, can give wrong results, see MultiBody.UsersGuide.Tutorial.ConnectionOfLineForces"));
  SI.Distance length "Distance between the origin of frame_a and the origin of frame_b";
  SI.Position r_rel_0[3] "Position vector from frame_a to frame_b resolved in world frame";
  Real e_rel_0[3](each final unit = "1") "Unit vector in direction from frame_a to frame_b, resolved in world frame";
protected
  SI.Force fa "Force from flange_a";
  SI.Force fb "Force from flange_b";
  SI.Position r_CM1_0[3](each stateSelect = StateSelect.avoid) "Position vector from world frame to point mass 1, resolved in world frame";
  SI.Position r_CM2_0[3](each stateSelect = StateSelect.avoid) "Position vector from world frame to point mass 2, resolved in world frame";
  Modelica.SIunits.Velocity v_CM1_0[3](each stateSelect = StateSelect.avoid) "der(r_CM_1_0) - velocity of point mass 1";
  Modelica.SIunits.Velocity v_CM2_0[3](each stateSelect = StateSelect.avoid) "der(r_CM_2_0) - velocity of point mass 2";
  Modelica.SIunits.Acceleration ag_CM1_0[3] "der(v_CM1_0) - gravityAcceleration(r_CM1_0)";
  Modelica.SIunits.Acceleration ag_CM2_0[3] "der(v_CM2_0) - gravityAcceleration(r_CM2_0)";
  SI.Force aux1_0[3] "Auxiliary force 1";
  SI.Force aux2_0[3] "Auxiliary force 2";
  input SI.Length cylinderDiameter_b = cylinderDiameter_a * diameterFraction;
  input SI.Length massDiameter = cylinderDiameter_a * massDiameterFaction;
  parameter Boolean animateMasses2 = world.enableAnimation and animate and animateMasses and m_a > 0 and m_b > 0;
  Visualizers.Advanced.Shape cylinder_a(shapeType = "cylinder", color = color_a, specularCoefficient = specularCoefficient, length = cylinderLength_a, width = cylinderDiameter_a, height = cylinderDiameter_a, lengthDirection = e_rel_0, widthDirection = {0, 1, 0}, r = frame_a.r_0) if world.enableAnimation and animate;
  Visualizers.Advanced.Shape cylinder_b(shapeType = "cylinder", color = color_b, specularCoefficient = specularCoefficient, length = cylinderLength_b, width = cylinderDiameter_b, height = cylinderDiameter_b, lengthDirection = -e_rel_0, widthDirection = {0, 1, 0}, r = frame_b.r_0) if world.enableAnimation and animate;
  Visualizers.Advanced.Shape sphere_a(shapeType = "sphere", color = massColor, specularCoefficient = specularCoefficient, length = massDiameter, width = massDiameter, height = massDiameter, lengthDirection = e_rel_0, widthDirection = {0, 1, 0}, r_shape = e_rel_0 * (L_a - massDiameter / 2), r = frame_a.r_0) if animateMasses2;
  Visualizers.Advanced.Shape sphere_b(shapeType = "sphere", color = massColor, specularCoefficient = specularCoefficient, length = massDiameter, width = massDiameter, height = massDiameter, lengthDirection = -e_rel_0, widthDirection = {0, 1, 0}, r_shape = -e_rel_0 * (L_b - massDiameter / 2), r = frame_b.r_0) if animateMasses2;
equation
  assert(noEvent(length > s_small), "
    The distance between the origin of frame_a and the origin of frame_b
    of a LineForceWithTwoMasses component became smaller as parameter s_small
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
  /* Force and torque balance of the two point masses
         - Kinematics for center of masses CM1, CM2 of point masses including gravity
           (L = length, va = der(frame_a.r_0), vb = der(frame_b.r_0))
           r_CM1_0 = frame_a.r_0 + e_rel_0*L_a;
           r_CM2_0 = frame_b.r_0 - e_rel_0*L_b;
           v_CM1_0 = der(r_CM1_0);
           v_CM2_0 = der(r_CM2_0);
           ag_CM1_0 = der(v_CM1_0) - world.gravityAcceleration(r_CM1_0);
           ag_CM2_0 = der(v_CM2_0) - world.gravityAcceleration(r_CM2_0);
           der(e_rel_0) = der(r_rel_0/sqrt(r_rel_0*r_rel_0))
                        = 1/L*(I - e_rel_0*e_rel_0')*der(r_rel_0)
                        = 1/L*(I - e_rel_0*e_rel_0')*(vb - va)
           v_CM1_0 = va + L_a/L*(I - e_rel_0*e_rel_0')*(vb - va)
           v_CM2_0 = vb - L_b/L*(I - e_rel_0*e_rel_0')*(vb - va)
         - Power balance for the connection line
           (f1=force on frame_a side, f2=force on frame_b side)
           0 = f1*va - m_a*ag_CM1*v_CM1 + f2*vb - m_b*ag_CM2*v_CM2
             = f1*va - m_a*ag_CM1*(va + L_a/L*(I - e_rel*e_rel')*(vb - va)) +
               f2*vb - m_b*ag_CM2*(vb - L_b/L*(I - e_rel*e_rel')*(vb - va))
             = (f1 - m_a*ag_CM1*(I - L_a/L*(I - e_rel*e_rel'))
                   - m_b*ag_CM2*(L_b/L*(I - e_rel*e_rel')))*va +
               (f2 - m_b*ag_CM2*(I - L_b/L*(I - e_rel_0*e_rel_0'))
                   - m_a*ag_CM1*(L_a/L*(I - e_rel*e_rel')))*vb
             = va*(f1 - m_a*ag_CM1 +
                   (m_a*ag_CM1*L_a/L - m_b*ag_CM2*L_b/L)*(I - e_rel*e_rel')) +
               vb*(f2 - m_b*ag_CM2 +
                   (m_b*ag_CM2*L_b/L - m_a*ag_CM1*L_a/L)*(I - e_rel*e_rel'))
           since va and vb are completely independent from other
           the parenthesis must vanish:
             f1 := m_a*ag_CM1 - (m_a*ag_CM1*L_a/L - m_b*ag_CM2*L_b/L)*(I - e_rel*e_rel')
             f2 := m_b*ag_CM2 + (m_a*ag_CM1*L_a/L - m_b*ag_CM2*L_b/L)*(I - e_rel*e_rel')
           or
             aux1 := ag_CM1*(m_a*L_a/L) - ag_CM2*(m_b*L_b/L);
             aux2 := aux1 - (aux1'*e_rel)*e_rel
             f1 := m_a*ag_CM1 - aux2
             f2 := m_b*ag_CM2 + aux2
         - Force balance on frame_a and frame_b finally results in
             0 = frame_a.f + e_rel_a*fa - f1_a
             0 = frame_b.f + e_rel_b*fb - f2_b
           and therefore
             frame_a.f = -e_rel_a*fa + m_a*ag_CM1 - aux2
             frame_b.f = -e_rel_b*fb + m_b*ag_CM2 + aux2
      */
  if m_a > 0 or m_b > 0 then
    r_CM1_0 = frame_a.r_0 + e_rel_0 * L_a;
    r_CM2_0 = frame_b.r_0 - e_rel_0 * L_b;
    v_CM1_0 = der(r_CM1_0);
    v_CM2_0 = der(r_CM2_0);
    ag_CM1_0 = der(v_CM1_0) - world.gravityAcceleration(r_CM1_0);
    ag_CM2_0 = der(v_CM2_0) - world.gravityAcceleration(r_CM2_0);
    aux1_0 = ag_CM1_0 * (m_a * L_a / length) - ag_CM2_0 * (m_b * L_b / length);
    aux2_0 = aux1_0 - aux1_0 * e_rel_0 * e_rel_0;
    frame_a.f = Frames.resolve2(frame_a.R, m_a * ag_CM1_0 - aux2_0 - e_rel_0 * fa);
    frame_b.f = Frames.resolve2(frame_b.R, m_b * ag_CM2_0 + aux2_0 - e_rel_0 * fb);
  else
    r_CM1_0 = zeros(3);
    r_CM2_0 = zeros(3);
    v_CM1_0 = zeros(3);
    v_CM2_0 = zeros(3);
    ag_CM1_0 = zeros(3);
    ag_CM2_0 = zeros(3);
    aux1_0 = zeros(3);
    aux2_0 = zeros(3);
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
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -95}, {150, -55}}, textString = "%name"), Line(visible = true, points = {{60, 0}, {60, 20}, {30, 20}, {30, 70}, {60, 70}, {60, 110}}, color = {64, 64, 64}), Line(visible = true, points = {{-23, 0}, {25, 0}}, pattern = LinePattern.Dot), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, extent = {{23, -8}, {39, 8}}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, extent = {{-39, -8}, {-23, 8}}), Line(visible = true, points = {{-60, 0}, {-29, 0}}, color = {64, 64, 64}), Ellipse(visible = fixedRotationAtFrame_a, lineColor = {255, 0, 0}, extent = {{-130, -30}, {-70, 30}}), Text(visible = fixedRotationAtFrame_a, textColor = {255, 0, 0}, extent = {{-140, 30}, {-62, 50}}, textString = "R=0"), Ellipse(visible = fixedRotationAtFrame_b, lineColor = {255, 0, 0}, extent = {{70, -30}, {130, 30}}), Text(visible = fixedRotationAtFrame_b, textColor = {255, 0, 0}, extent = {{62, 30}, {140, 50}}, textString = "R=0"), Line(visible = true, points = {{29, 0}, {60, 0}}, color = {64, 64, 64}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{-70, -15}, {-40, 15}}), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-40.019, 25.992}, {-40.019, 25.992}, {-47.214, 28.972}, {-55, 30}, {-62.786, 28.972}, {-69.981, 25.992}, {-76.213, 21.213}, {-80.992, 14.981}, {-83.972, 7.786}, {-85, 0}, {-83.972, -7.786}, {-80.992, -14.981}, {-76.213, -21.213}, {-69.981, -25.992}, {-62.786, -28.972}, {-55, -30}, {-47.214, -28.972}, {-40.019, -25.992}, {-40.019, -25.992}, {-39.693, -36.955}, {-39.693, -36.955}, {-47.16, -39.224}, {-55, -40}, {-62.84, -39.224}, {-70.307, -36.955}, {-77.192, -33.279}, {-83.284, -28.284}, {-88.279, -22.192}, {-91.955, -15.307}, {-94.224, -7.84}, {-95, 0}, {-94.224, 7.84}, {-91.955, 15.307}, {-88.279, 22.192}, {-83.284, 28.284}, {-77.192, 33.279}, {-70.307, 36.955}, {-62.84, 39.224}, {-55, 40}, {-47.16, 39.224}, {-39.693, 36.955}, {-39.693, 36.955}}, smooth = Smooth.Bezier), Polygon(visible = true, rotation = 180, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-40.019, 25.992}, {-40.019, 25.992}, {-47.214, 28.972}, {-55, 30}, {-62.786, 28.972}, {-69.981, 25.992}, {-76.213, 21.213}, {-80.992, 14.981}, {-83.972, 7.786}, {-85, 0}, {-83.972, -7.786}, {-80.992, -14.981}, {-76.213, -21.213}, {-69.981, -25.992}, {-62.786, -28.972}, {-55, -30}, {-47.214, -28.972}, {-40.019, -25.992}, {-40.019, -25.992}, {-39.693, -36.955}, {-39.693, -36.955}, {-47.16, -39.224}, {-55, -40}, {-62.84, -39.224}, {-70.307, -36.955}, {-77.192, -33.279}, {-83.284, -28.284}, {-88.279, -22.192}, {-91.955, -15.307}, {-94.224, -7.84}, {-95, 0}, {-94.224, 7.84}, {-91.955, 15.307}, {-88.279, 22.192}, {-83.284, 28.284}, {-77.192, 33.279}, {-70.307, 36.955}, {-62.84, 39.224}, {-55, 40}, {-47.16, 39.224}, {-39.693, 36.955}, {-39.693, 36.955}}, smooth = Smooth.Bezier), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{40, -15}, {70, 15}}), Line(visible = true, points = {{-60, 0}, {-60, 23}, {-30, 23}, {-30, 70}, {-60, 70}, {-60, 110}}, color = {64, 64, 64})}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{-60, 80}, {60, 80}}, color = {64, 64, 64}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 15), Text(visible = true, origin = {6.667, 5}, textColor = {64, 64, 64}, extent = {{-46.667, 79}, {33.333, 91}}, textString = "length"), Line(visible = true, points = {{-60, 0}, {-60, 24}, {-40, 24}, {-40, 60}, {-60, 60}, {-60, 110}}, color = {64, 64, 64}), Line(visible = true, points = {{60, 1}, {60, 21}, {40, 21}, {40, 60}, {60, 60}, {60, 110}}, color = {64, 64, 64}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, extent = {{20, -8}, {36, 8}}), Line(visible = true, points = {{-18, -18}, {11, -18}}, color = {64, 64, 64}), Polygon(visible = true, lineColor = {64, 64, 64}, points = {{23, -18}, {11, -15}, {11, -21}, {23, -18}}), Line(visible = true, points = {{-60, 16}, {-37, 16}}, color = {64, 64, 64}), Line(visible = true, points = {{-25, 0}, {-25, 20}}, color = {64, 64, 64}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-38, -35}, {33, -20}}, textString = "e_rel_0"), Polygon(visible = true, lineColor = {64, 64, 64}, points = {{-25, 16}, {-37, 19}, {-37, 13}, {-25, 16}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-39, 21}, {-22, 31}}, textString = "L_a"), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, extent = {{-33, -9}, {-17, 7}}), Line(visible = true, points = {{28, 7.57}, {28, 20}}, color = {64, 64, 64}), Line(visible = true, points = {{40, 16}, {60, 16}}, color = {64, 64, 64}), Polygon(visible = true, lineColor = {64, 64, 64}, points = {{28, 16}, {40, 19}, {40, 13}, {28, 16}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{15, 26}, {32, 36}}, textString = "L_b"), Line(visible = true, points = {{37, 18}, {30, 27}}, color = {64, 64, 64}, pattern = LinePattern.Dot), Line(visible = true, points = {{-60, 0}, {60, 0}}, color = {64, 64, 64}, pattern = LinePattern.Dot), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{45, -15}, {75, 15}}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{-75.846, -15}, {-45.846, 15}}), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-45.019, 25.992}, {-45.019, 25.992}, {-52.214, 28.972}, {-60, 30}, {-67.786, 28.972}, {-74.981, 25.992}, {-81.213, 21.213}, {-85.992, 14.981}, {-88.972, 7.786}, {-90, 0}, {-88.972, -7.786}, {-85.992, -14.981}, {-81.213, -21.213}, {-74.981, -25.992}, {-67.786, -28.972}, {-60, -30}, {-52.214, -28.972}, {-45.019, -25.992}, {-45.019, -25.992}, {-44.693, -36.955}, {-44.693, -36.955}, {-52.16, -39.224}, {-60, -40}, {-67.84, -39.224}, {-75.307, -36.955}, {-82.192, -33.279}, {-88.284, -28.284}, {-93.279, -22.192}, {-96.955, -15.307}, {-99.224, -7.84}, {-100, 0}, {-99.224, 7.84}, {-96.955, 15.307}, {-93.279, 22.192}, {-88.284, 28.284}, {-82.192, 33.279}, {-75.307, 36.955}, {-67.84, 39.224}, {-60, 40}, {-52.16, 39.224}, {-44.693, 36.955}, {-44.693, 36.955}}, smooth = Smooth.Bezier), Polygon(visible = true, rotation = 180, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-45.019, 25.992}, {-45.019, 25.992}, {-52.214, 28.972}, {-60, 30}, {-67.786, 28.972}, {-74.981, 25.992}, {-81.213, 21.213}, {-85.992, 14.981}, {-88.972, 7.786}, {-90, 0}, {-88.972, -7.786}, {-85.992, -14.981}, {-81.213, -21.213}, {-74.981, -25.992}, {-67.786, -28.972}, {-60, -30}, {-52.214, -28.972}, {-45.019, -25.992}, {-45.019, -25.992}, {-44.693, -36.955}, {-44.693, -36.955}, {-52.16, -39.224}, {-60, -40}, {-67.84, -39.224}, {-75.307, -36.955}, {-82.192, -33.279}, {-88.284, -28.284}, {-93.279, -22.192}, {-96.955, -15.307}, {-99.224, -7.84}, {-100, 0}, {-99.224, 7.84}, {-96.955, 15.307}, {-93.279, 22.192}, {-88.284, 28.284}, {-82.192, 33.279}, {-75.307, 36.955}, {-67.84, 39.224}, {-60, 40}, {-52.16, 39.224}, {-44.693, 36.955}, {-44.693, 36.955}}, smooth = Smooth.Bezier)}), Documentation(info = "<html>
<p>
This component is used to exert a <b>line force</b>
between the origin of frame_a and the origin of frame_b
by attaching components of the <b>1-dimensional translational</b>
mechanical library of Modelica (Modelica.Mechanics.Translational)
between the two flange connectors <b>flange_a</b> and
<b>flange_b</b>. Optionally, there are <b>two point masses</b> on the line
connecting the origin of frame_a and the origin of frame_b.
These point masses approximate the <b>masses</b> of the <b>force element</b>.
The locations of the two point masses are defined by their
(fixed) distances of L_a relative to frame_a and of L_b relative
to frame_b, respectively.
</p>
<p>
In example
<a href=\"modelica://UWBody.Examples.Elementary.LineForceWithTwoMasses\">
MultiBody.Examples.Elementary.LineForceWithTwoMasses</a> the usage of this
line force element is shown and is compared with an alternative
implementation using a
<a href=\"modelica://UWBody.Joints.Assemblies.JointUPS\">
MultiBody.Joints.Assemblies.JointUPS</a> component.
The composition diagram of this example
is displayed in the figure below.
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/LineForceWithTwoMasses1.png\">
</p>

<p>
The animation view at time = 0 is shown in the next figure.
The system on the left side in the front is the animation with
the LineForceWithTwoMasses component whereas the system on the right
side in the back is the animation with the JointUPS component.
Both implementations yield the same result. However, the implementation
with the LineForceWithTwoMasses component is simpler.
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/LineForceWithTwoMasses2.png\">
</p>

<p>
In the translational library there is the implicit assumption that
forces of components that have only one flange connector act with
opposite sign on the bearings of the component. This assumption
is also used in the LineForceWithTwoMasses component: If a connection
is present to only one of the flange connectors, then the force
in this flange connector acts implicitly with opposite sign also
in the other flange connector.
</p>
</html>"));
end LineForceWithTwoMasses;
