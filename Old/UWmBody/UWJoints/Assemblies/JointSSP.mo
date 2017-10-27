within UWmBody.UWJoints.Assemblies;

model JointSSP "Spherical - spherical - prismatic joint aggregation with mass (no constraints, no potential states)"
  import UWmBody.UWTypes;
  extends UWInterfaces.PartialTwoFramesDoubleSize;
  UWmBody.UWInterfaces.Frame_b frame_ib "Coordinate system at origin of frame_b fixed at connecting rod of spherical and prismatic joint" annotation(Placement(transformation(origin = {80, 100}, extent = {{-8, 8}, {8, -8}}, rotation = 270), visible = true, iconTransformation(origin = {80, 100}, extent = {{-8, 8}, {8, -8}}, rotation = 270)));
  UWmBody.UWInterfaces.Frame_b frame_im "Coordinate system at origin of spherical joint in the middle fixed at connecting rod of spherical and prismatic joint" annotation(Placement(transformation(origin = {0, 100}, extent = {{8, -8}, {-8, 8}}, rotation = 270), visible = true, iconTransformation(origin = {0, 100}, extent = {{8, -8}, {-8, 8}}, rotation = 270)));
  Modelica.Mechanics.Translational.Interfaces.Flange_a axis "1-dim. translational flange that drives the prismatic joint" annotation(Placement(transformation(extent = {{95, 75}, {105, 85}})));
  Modelica.Mechanics.Translational.Interfaces.Flange_b bearing "1-dim. translational flange of the drive bearing of the prismatic joint" annotation(Placement(transformation(extent = {{105, 35}, {95, 45}}, origin = {0, 0}, rotation = 0), visible = true, iconTransformation(origin = {0, 0}, extent = {{105, 35}, {95, 45}}, rotation = 0)));
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter Boolean showMass = true "= true, if point mass on rod 1 shall be shown (provided animation = true and rod1Mass > 0)";
  parameter SI.Length rod1Length(min = Modelica.Constants.eps, start = 1) "Distance between the origins of the two spherical joints";
  parameter SI.Mass rod1Mass(min = 0) = 0 "Mass of rod 1 (= point mass located in middle of rod connecting the two spherical joints)";
  parameter UWmBody.UWTypes.Axis n_b = {0, 0, 1} "Axis of prismatic joint fixed and resolved in frame_b";
  parameter SI.Position rRod2_ib[3] = {1, 0, 0} "Vector from origin of frame_ib to spherical joint in the middle, resolved in frame_ib";
  parameter SI.Position s_offset = 0 "Relative distance offset of prismatic joint (distance between frame_b and frame_ib = s(t) + s_offset)";
  parameter SI.Position s_guess = 0 "Select the configuration such that at initial time |s(t0)-s_guess| is minimal";
  parameter SI.Diameter sphereDiameter = world.defaultJointLength "Diameter of the spheres representing the two spherical joints" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.Color sphereColor = UWmBody.UWTypes.Defaults.JointColor "Color of the spheres representing the two spherical joints" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Diameter rod1Diameter = sphereDiameter / Types.Defaults.JointRodDiameterFraction "Diameter of rod 1 connecting the two spherical joints" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.Color rod1Color = UWmBody.UWTypes.Defaults.RodColor "Color of rod 1 connecting the two spherical joint" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Diameter rod2Diameter = rod1Diameter "Diameter of rod 2 connecting the revolute joint and spherical joint 2" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.Color rod2Color = rod1Color "Color of rod 2 connecting the revolute joint and spherical joint 2" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  parameter UWTypes.Axis boxWidthDirection = {0, 1, 0} "Vector in width direction of prismatic joint box, resolved in frame_b" annotation(Evaluate = true, Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Distance boxWidth = world.defaultJointWidth "Width of prismatic joint box" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Distance boxHeight = boxWidth "Height of prismatic joint box" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.Color boxColor = UWmBody.UWTypes.Defaults.JointColor "Color of prismatic joint box" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter Boolean checkTotalPower = false "= true, if total power flowing into this component shall be determined (must be zero)" annotation(Dialog(tab = "Advanced"));
  Real aux "Denominator used to compute force in rod connecting universal and spherical joint";
  SI.Force f_rod "Constraint force in direction of the rod (positive, if rod is pressed)";
  SI.Power totalPower "Total power flowing into this element, if checkTotalPower=true (otherwise dummy)";
  UWmBody.UWJoints.Internal.PrismaticWithLengthConstraint prismatic(animation = animation, length = rod1Length, n = n_b, s_offset = s_offset, s_guess = s_guess, boxWidthDirection = boxWidthDirection, boxWidth = boxWidth, boxHeight = boxHeight, specularCoefficient = specularCoefficient, boxColor = boxColor) annotation(Placement(transformation(extent = {{75, -20}, {35, 20}})));
  UWmBody.UWJoints.SphericalSpherical rod1(animation = animation, showMass = showMass, m = rod1Mass, rodLength = rod1Length, rodDiameter = rod1Diameter, sphereDiameter = sphereDiameter, rodColor = rod1Color, kinematicConstraint = false, specularCoefficient = specularCoefficient, sphereColor = sphereColor, constraintResidue = rod1.f_rod - f_rod) annotation(Placement(transformation(extent = {{-89, -20}, {-49, 20}})));
  UWmBody.UWParts.FixedTranslation rod2(animation = animation, width = rod2Diameter, height = rod2Diameter, specularCoefficient = specularCoefficient, color = rod2Color, r = rRod2_ib) annotation(Placement(transformation(extent = {{15, -20}, {-25, 20}})));
  UWSensors.RelativePosition relativePosition(resolveInFrame = UWmBody.UWTypes.ResolveInFrameAB.frame_a) annotation(Placement(transformation(extent = {{60, -70}, {40, -90}})));
  Modelica.Blocks.Sources.Constant position_b[3](k = rRod2_ib) annotation(Placement(transformation(extent = {{-20, -50}, {0, -30}})));
equation
  /* Compute the unknown force in the rod of the rod1 joint
           by a force balance:
             0 = frame_b.f + frame_ib.f + frame_im.f +
                 Frames.resolve2(rod1.R_rel, rod1.f_rod*rod1.eRod_a)
           The condition is that the projection of the force in the prismatic
           joint along the axis of the prismatic joint is equal to the driving
           axis force in the flange:
             -prismatic.f = prismatic.e*frame_b.f
           Therefore, we have with e=prismatic.e and f=prismatic.f
              f = e*(frame_ib.f + frame_im.f +
                     Frames.resolve2(rod1.R_rel, rod1.f_rod*rod1.eRod_a))
                = e*(frame_ib.f + frame_im.f +
                     rod1.f_rod*Frames.resolve2(rod1.R_rel, rod1.eRod_a))
           Solving this equation for f_rod results in
             rod1.f_rod = (f - e*(frame_ib.f + frame_im.f))
                          / (e*Frames.resolve2(rod1.R_rel, rod1.eRod_a))
           Additionally, a guard against division by zero is introduced
        */
  aux = prismatic.e * Frames.resolveRelative(rod1.eRod_a, rod1.frame_a.R, rod1.frame_b.R);
  f_rod = ((-prismatic.f) - prismatic.e * (frame_ib.f + frame_im.f)) / noEvent(if abs(aux) < 1.e-10 then 1.e-10 else aux);
  // Measure power for test purposes
  if checkTotalPower then
    totalPower = frame_a.f * Frames.resolve2(frame_a.R, der(frame_a.r_0)) + frame_b.f * Frames.resolve2(frame_b.R, der(frame_b.r_0)) + frame_ib.f * Frames.resolve2(frame_ib.R, der(frame_ib.r_0)) + frame_im.f * Frames.resolve2(frame_im.R, der(frame_im.r_0)) + frame_a.t * Frames.angularVelocity2(frame_a.R) + frame_b.t * Frames.angularVelocity2(frame_b.R) + frame_ib.t * Frames.angularVelocity2(frame_ib.R) + frame_im.t * Frames.angularVelocity2(frame_im.R) + axis.f * der(axis.s) + bearing.f * der(bearing.s) + (-rod1Mass) * (der(rod1.v_CM_0) - world.gravityAcceleration(rod1.r_CM_0)) * rod1.v_CM_0;
  else
    totalPower = 0;
  end if;
  connect(prismatic.frame_b, rod2.frame_a) annotation(Line(points = {{35, 0}, {15, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(rod2.frame_b, rod1.frame_b) annotation(Line(points = {{-25, 0}, {-49, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(prismatic.frame_a, frame_b) annotation(Line(points = {{75, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(rod2.frame_a, frame_ib) annotation(Line(points = {{15, 0}, {26, 0}, {26, 70}, {80, 70}, {80, 100}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(rod1.frame_a, frame_a) annotation(Line(points = {{-89, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(relativePosition.frame_b, frame_a) annotation(Line(points = {{40, -80}, {-95, -80}, {-95, 0}, {-100, 0}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(relativePosition.frame_a, frame_b) annotation(Line(points = {{60, -80}, {96, -80}, {96, 0}, {100, 0}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(position_b.y, prismatic.position_b) annotation(Line(points = {{1, -40}, {20, -40}, {20, -12}, {31, -12}}, color = {0, 0, 127}));
  connect(prismatic.axis, axis) annotation(Line(points = {{39, 14}, {40, 14}, {40, 60}, {90, 60}, {90, 80}, {100, 80}}));
  connect(prismatic.bearing, bearing) annotation(Line(points = {{63, 14}, {63, 40}, {100, 40}}, visible = true));
  connect(rod2.frame_b, frame_im) annotation(Line(points = {{-25, 0}, {-35, 0}, {-35, 60}, {0, 60}, {0, 100}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(relativePosition.r_rel, prismatic.position_a) annotation(Line(points = {{50, -69}, {50, -50}, {90, -50}, {90, -12}, {79, -12}}, color = {0, 0, 127}));
  annotation(Documentation(info = "<html>
<p>
This component consists of a <b>spherical</b> joint 1 at frame_a, a <b>prismatic</b>
joint at frame_b and a <b>spherical</b> joint 2 which is connected via rod 1
to the spherical joint 1 and via rod 2 to the prismatic joint, see the default
animation in the following figure (the axes vectors are not part of the
default animation):
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/JointSSP.png\" ALT=\"model Joints.Assemblies.JointSSP\">
</p>

<p>
Besides an optional point mass in the middle of rod 1,
this joint aggregation has no mass and no inertia,
and introduces neither constraints nor potential state variables.
It should be used in kinematic loops whenever possible since
the non-linear system of equations introduced by this joint aggregation
is solved <b>analytically</b> (i.e., a solution is always computed, if a
unique solution exists).
</p>
<p>
An additional <b>frame_ib</b> is present. It is <b>fixed</b> in rod 2
connecting the prismatic and the spherical joint at the side of the prismatic
joint that is connected to this rod (= rod2.frame_a = prismatic.frame_a).
</p>
<p>
An additional <b>frame_im</b> is present. It is <b>fixed</b> in rod 2
connecting the prismatic and the spherical joint at the side of spherical
joint 2 that is connected to this rod (= rod2.frame_b).
It is always parallel to <b>frame_ib</b>.
</p>
<p>
The easiest way to define the parameters of this joint is by moving the
MultiBody system in a <b>reference configuration</b> where <b>all frames</b>
of all components are <b>parallel</b> to each other (alternatively,
at least frame_b and frame_ib of the JointSSP joint
should be parallel to each other when defining an instance of this
component).
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.2, grid = {5, 5}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-140, -90}, {140, -50}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{89, 90}, {132, 110}}, textString = "ib"), Line(visible = true, points = {{20, 0}, {20, 80}, {0, 80}, {0, 100}}, color = {95, 95, 95}, thickness = 0.5), Text(visible = true, textColor = {64, 64, 64}, extent = {{-49, 90}, {-11, 110}}, textString = "im"), Line(visible = true, points = {{50, 5}, {50, 80}, {80, 80}, {80, 100}}, color = {95, 95, 95}, thickness = 0.5), Line(visible = true, points = {{101, 80}, {80, 80}}, color = {95, 95, 95}, thickness = 0.5), Line(visible = true, points = {{100, 40}, {90, 40}, {90, 20}}, color = {95, 95, 95}, thickness = 0.5), Rectangle(visible = true, origin = {-40, 0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-30, -5}, {30, 5}}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{-80, -9}, {-60, 11}}), Rectangle(visible = true, origin = {37.5, 0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-22.5, -5}, {22.5, 5}}), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-58.764, 19.494}, {-58.764, 19.494}, {-64.161, 21.729}, {-70, 22.5}, {-75.839, 21.729}, {-81.236, 19.494}, {-85.91, 15.91}, {-89.494, 11.236}, {-91.729, 5.84}, {-92.5, 0}, {-91.729, -5.839}, {-89.494, -11.236}, {-85.91, -15.91}, {-81.236, -19.494}, {-75.839, -21.729}, {-70, -22.5}, {-64.161, -21.729}, {-58.764, -19.494}, {-58.764, -19.494}, {-58.52, -27.716}, {-58.52, -27.716}, {-64.12, -29.418}, {-70, -30}, {-75.88, -29.418}, {-81.48, -27.716}, {-86.644, -24.959}, {-91.213, -21.213}, {-94.959, -16.644}, {-97.716, -11.48}, {-99.418, -5.88}, {-100, 0}, {-99.418, 5.88}, {-97.716, 11.48}, {-94.959, 16.644}, {-91.213, 21.213}, {-86.644, 24.959}, {-81.48, 27.716}, {-75.88, 29.418}, {-70, 30}, {-64.12, 29.418}, {-58.52, 27.716}, {-58.52, 27.716}}, smooth = Smooth.Bezier), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{-20, -10}, {0, 10}}), Polygon(visible = true, origin = {-10, 0}, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-11.236, 19.494}, {-11.236, 19.494}, {-5.84, 21.729}, {0, 22.5}, {5.84, 21.729}, {11.236, 19.494}, {15.91, 15.91}, {19.494, 11.236}, {21.729, 5.84}, {22.5, 0}, {21.729, -5.84}, {19.494, -11.236}, {15.91, -15.91}, {11.236, -19.494}, {5.84, -21.729}, {0, -22.5}, {-5.84, -21.729}, {-11.236, -19.494}, {-11.236, -19.494}, {-11.48, -27.716}, {-11.48, -27.716}, {-5.88, -29.418}, {0, -30}, {5.88, -29.418}, {11.48, -27.716}, {16.644, -24.959}, {21.213, -21.213}, {24.959, -16.644}, {27.716, -11.48}, {29.418, -5.88}, {30, 0}, {29.418, 5.88}, {27.716, 11.48}, {24.959, 16.644}, {21.213, 21.213}, {16.644, 24.959}, {11.48, 27.716}, {5.88, 29.418}, {0, 30}, {-5.88, 29.418}, {-11.48, 27.716}, {-11.48, 27.716}}, smooth = Smooth.Bezier), Rectangle(visible = true, origin = {90, 0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, extent = {{-10, -20}, {10, 20}}), Rectangle(visible = true, origin = {75, 0}, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, extent = {{-5, -20}, {5, 20}}), Rectangle(visible = true, origin = {62.5, 0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, extent = {{-12.5, -10}, {12.5, 10}})}));
end JointSSP;
