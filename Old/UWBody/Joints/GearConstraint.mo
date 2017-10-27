within UWBody.Joints;

model GearConstraint "Ideal 3-dim. gearbox (arbitrary shaft directions)"
  import UWBody.Frames;
  extends UWBody.Interfaces.PartialTwoFrames;
  Interfaces.Frame_a bearing "Coordinate system fixed in the bearing" annotation(Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  parameter Real ratio(start = 2) "Gear speed ratio";
  parameter UWBody.Types.Axis n_a = {1, 0, 0} "Axis of rotation of shaft a (same coordinates in frame_a, frame_b, bearing)";
  parameter UWBody.Types.Axis n_b = {1, 0, 0} "Axis of rotation of shaft b (same coordinates in frame_a, frame_b, bearing)";
  parameter Modelica.SIunits.Position r_a[3] = {0, 0, 0} "Vector from frame bearing to frame_a resolved in bearing";
  parameter Modelica.SIunits.Position r_b[3] = {0, 0, 0} "Vector from frame bearing to frame_b resolved in bearing";
  parameter StateSelect stateSelect = StateSelect.default "Priority to use joint coordinates (phi_a, phi_b, w_a, w_b) as states" annotation(Dialog(tab = "Advanced"));
  parameter Boolean checkTotalPower = false "= true, if total power flowing into this component shall be determined (must be zero)" annotation(Dialog(tab = "Advanced"));
  Modelica.SIunits.Angle phi_b(start = 0, stateSelect = stateSelect) "Relative rotation angle of revolute joint at frame_b";
  SI.AngularVelocity w_b(start = 0, stateSelect = stateSelect) "First derivative of angle phi_b (relative angular velocity b)";
  SI.AngularAcceleration a_b(start = 0) "Second derivative of angle phi_b (relative angular acceleration b)";
  SI.Power totalPower "Total power flowing into this element, if checkTotalPower=true (otherwise dummy)";
  UWBody.Joints.Revolute actuatedRevolute_a(useAxisFlange = true, n = n_a, animation = false) annotation(Placement(transformation(extent = {{-40, -10}, {-60, 10}})));
  UWBody.Joints.Revolute actuatedRevolute_b(useAxisFlange = true, n = n_b, animation = false) annotation(Placement(transformation(extent = {{40, -10}, {60, 10}})));
  Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio = ratio) annotation(Placement(transformation(extent = {{-10, 30}, {10, 50}})));
  UWBody.Parts.FixedTranslation fixedTranslation1(animation = false, r = r_b) annotation(Placement(transformation(extent = {{10, -10}, {30, 10}})));
  UWBody.Parts.FixedTranslation fixedTranslation2(animation = false, r = r_a) annotation(Placement(transformation(origin = {-20, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
equation
  /* Implementation notes:
    
         The GearConstraint model consists primarily of two revolute joints that are
         connected together and to a support/mounting. In this first phase the two
         revolute joints can be rotated independently from each other and therefore
         there are two degrees of freedom. If the rotational angles of these joints
         would be used as generalized coordinates phi_a, phi_b with associated generalized
         torques tau_a, tau_b (torques along the axes of rotations), then the equations
         of motion (Kanes' equations or Lagranges' equations of the second kind) are
         in the rows for phi_a, phi_b:
            .... = ... + {...., tau_a, tau_b, ....}
    
         Now the kinematic constraint for the gear is added:
    
             0 = phi_a - ratio*phi_b;
    
         or on velocity level:
    
             0 = G * {der(phi_a), der(phi_b)};   G = [1, -ratio]
    
         According to Lagranges' equations of the first kind, the generalized forces
         must be replaced by G'*lambda, where lambda is the new constraint force
         due this constraint. Therefore, the equations of motions are changed to
    
           .... = .... + {...., G'*lambda, .....}
    
         This is equivalent to add the equations
    
           tau_a = lambda
           tau_b = -ratio*lambda
    
         or
    
           0 = tau_b + ratio*tau_a;
    
         The two equations
    
           0 = phi_a - ratio*phi_b
           0 = tau_b + ratio*tau_a
    
         are completely identical to the equations of an ideal gear (without mounting)
         that connects the axis flanges of the two revolute joints. This in turn
         means that the two rotational flanges of the revolute joints just have to be
         connected by an IdealGear component (without mounting).
      */
  assert(cardinality(bearing) > 0, "Connector bearing of component is not connected");
  phi_b = actuatedRevolute_b.phi;
  w_b = der(phi_b);
  a_b = der(w_b);
  // Measure power for test purposes
  if checkTotalPower then
    totalPower = frame_a.f * Frames.resolve2(frame_a.R, der(frame_a.r_0)) + frame_b.f * Frames.resolve2(frame_b.R, der(frame_b.r_0)) + bearing.f * Frames.resolve2(bearing.R, der(bearing.r_0)) + frame_a.t * Frames.angularVelocity2(frame_a.R) + frame_b.t * Frames.angularVelocity2(frame_b.R) + bearing.t * Frames.angularVelocity2(bearing.R);
  else
    totalPower = 0;
  end if;
  connect(actuatedRevolute_a.axis, idealGear.flange_a) annotation(Line(points = {{-50, 10}, {-50, 40}, {-10, 40}}));
  connect(idealGear.flange_b, actuatedRevolute_b.axis) annotation(Line(points = {{10, 40}, {50, 40}, {50, 10}}));
  connect(actuatedRevolute_a.frame_a, fixedTranslation2.frame_b) annotation(Line(points = {{-40, 0}, {-35, 0}, {-30, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(fixedTranslation2.frame_a, bearing) annotation(Line(points = {{-10, 0}, {-4, 0}, {0, 0}, {0, -100}}, color = {95, 95, 95}, thickness = 0.5));
  connect(fixedTranslation1.frame_a, bearing) annotation(Line(points = {{10, 0}, {0, 0}, {0, -100}}, color = {95, 95, 95}, thickness = 0.5));
  connect(fixedTranslation1.frame_b, actuatedRevolute_b.frame_a) annotation(Line(points = {{30, 0}, {40, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(frame_a, actuatedRevolute_a.frame_b) annotation(Line(points = {{-100, 0}, {-80, 0}, {-80, 0}, {-60, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(actuatedRevolute_b.frame_b, frame_b) annotation(Line(points = {{60, 0}, {80, 0}, {80, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, origin = {0, -20}, textColor = {64, 64, 64}, extent = {{-150, 135}, {150, 175}}, textString = "%name"), Text(visible = true, origin = {0, 12}, textColor = {64, 64, 64}, extent = {{-150, -94}, {150, -64}}, textString = "%ratio"), Rectangle(visible = true, origin = {-35, 60}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-15, -40}, {15, 40}}), Rectangle(visible = true, origin = {-35, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-15, -21}, {15, 21}}), Line(visible = true, points = {{-80, 20}, {-60, 20}}), Line(visible = true, points = {{-80, -20}, {-60, -20}}), Line(visible = true, points = {{-70, -20}, {-70, -86}}), Line(visible = true, points = {{0, 40}, {0, -100}}), Line(visible = true, points = {{-10, 40}, {10, 40}}), Line(visible = true, points = {{-10, 80}, {10, 80}}), Line(visible = true, points = {{60, -20}, {80, -20}}), Line(visible = true, points = {{60, 20}, {80, 20}}), Line(visible = true, points = {{70, -20}, {70, -86}}), Rectangle(visible = true, origin = {-75, 0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-25, -10}, {25, 10}}), Rectangle(visible = true, origin = {75, 0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-25, -10}, {25, 10}}), Rectangle(visible = true, origin = {-35, -19}, fillColor = {153, 153, 153}, fillPattern = FillPattern.Solid, extent = {{-15, -2}, {15, 2}}), Rectangle(visible = true, origin = {-35, -8}, fillColor = {204, 204, 204}, fillPattern = FillPattern.Solid, extent = {{-15, -3}, {15, 3}}), Rectangle(visible = true, origin = {-35, 19}, fillColor = {204, 204, 204}, fillPattern = FillPattern.Solid, extent = {{-15, -2}, {15, 2}}), Rectangle(visible = true, origin = {-35, 8}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-15, -3}, {15, 3}}), Rectangle(visible = true, origin = {0, 60}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-20, -10}, {20, 10}}), Rectangle(visible = true, origin = {-35, 98}, fillColor = {153, 153, 153}, fillPattern = FillPattern.Solid, extent = {{-15, -2}, {15, 2}}), Rectangle(visible = true, origin = {-35, 87}, fillColor = {204, 204, 204}, fillPattern = FillPattern.Solid, extent = {{-15, -3}, {15, 3}}), Rectangle(visible = true, origin = {-35, 50}, fillColor = {204, 204, 204}, fillPattern = FillPattern.Solid, extent = {{-15, -4}, {15, 4}}), Rectangle(visible = true, origin = {-35, 22}, fillColor = {102, 102, 102}, fillPattern = FillPattern.Solid, extent = {{-15, -2}, {15, 2}}), Rectangle(visible = true, origin = {-35, 33}, fillColor = {153, 153, 153}, fillPattern = FillPattern.Solid, extent = {{-15, -3}, {15, 3}}), Rectangle(visible = true, origin = {-35, 70}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-15, -4}, {15, 4}}), Rectangle(visible = true, origin = {35, 60}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-15, -21}, {15, 21}}), Rectangle(visible = true, origin = {35, 41}, fillColor = {153, 153, 153}, fillPattern = FillPattern.Solid, extent = {{-15, -2}, {15, 2}}), Rectangle(visible = true, origin = {35, 52}, fillColor = {204, 204, 204}, fillPattern = FillPattern.Solid, extent = {{-15, -3}, {15, 3}}), Rectangle(visible = true, origin = {35, 79}, fillColor = {204, 204, 204}, fillPattern = FillPattern.Solid, extent = {{-15, -2}, {15, 2}}), Rectangle(visible = true, origin = {35, 68}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-15, -3}, {15, 3}}), Rectangle(visible = true, origin = {35, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-15, -40}, {15, 40}}), Rectangle(visible = true, origin = {35, 38}, fillColor = {153, 153, 153}, fillPattern = FillPattern.Solid, extent = {{-15, -2}, {15, 2}}), Rectangle(visible = true, origin = {35, 27}, fillColor = {204, 204, 204}, fillPattern = FillPattern.Solid, extent = {{-15, -3}, {15, 3}}), Rectangle(visible = true, origin = {35, -10}, fillColor = {204, 204, 204}, fillPattern = FillPattern.Solid, extent = {{-15, -4}, {15, 4}}), Rectangle(visible = true, origin = {35, -27}, fillColor = {153, 153, 153}, fillPattern = FillPattern.Solid, extent = {{-15, -3}, {15, 3}}), Rectangle(visible = true, origin = {35, 10}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-15, -4}, {15, 4}}), Rectangle(visible = true, origin = {-35, 40}, fillColor = {255, 255, 255}, extent = {{-15, -61}, {15, 60}}), Rectangle(visible = true, origin = {35, 21}, fillColor = {255, 255, 255}, extent = {{-15, -61}, {15, 60}}), Line(visible = true, points = {{70, -86}, {-70, -86}}), Rectangle(visible = true, origin = {70, 58.284}, rotation = 45, lineColor = {190, 20, 31}, fillColor = {190, 20, 31}, fillPattern = FillPattern.Solid, extent = {{-20, -20}, {20, 20}}), Line(visible = true, origin = {70, 58.284}, points = {{-20, -5}, {-7.5, 7.5}, {7.5, -7.5}, {20, 5}}, color = {255, 255, 255}, thickness = 5)}), Documentation(info = "<html>
<p>This ideal massless joint provides a gear constraint between
frames <code>frame_a</code> and <code>frame_b</code>. The axes of rotation
of <code>frame_a</code> and <code>frame_b</code> may be arbitrary.</p>
<p><b>Reference</b><br>
<span style=\"font-variant:small-caps\">Schweiger</span>, Christian ;
<span style=\"font-variant:small-caps\">Otter</span>, Martin:
<a href=\"https://www.modelica.org/events/Conference2003/papers/h06_Schweiger_powertrains_v5.pdf\">Modelling
3D Mechanical Effects of 1-dim. Powertrains</a>. In: <i>Proceedings of the 3rd International
Modelica Conference</i>. Link&ouml;ping : The Modelica Association and Link&ouml;ping University,
November 3-4, 2003, pp. 149-158</p>
</html>"));
end GearConstraint;
