within UWBody.Joints.Assemblies;

model JointUPS "Universal - prismatic - spherical joint aggregation (no constraints, no potential states)"
  import UWBody.Types;
  extends Interfaces.PartialTwoFramesDoubleSize;
  UWBody.Interfaces.Frame_a frame_ia "Coordinate system at origin of frame_a fixed at prismatic joint" annotation(Placement(transformation(origin = {-80, 100}, extent = {{-8, -8}, {8, 8}}, rotation = 270), visible = true, iconTransformation(origin = {0, -0}, extent = {{-108, -88}, {-92, -72}}, rotation = 270)));
  UWBody.Interfaces.Frame_b frame_ib "Coordinate system at origin of frame_b fixed at prismatic joint" annotation(Placement(transformation(origin = {80, 100}, extent = {{-8, 8}, {8, -8}}, rotation = 270), visible = true, iconTransformation(origin = {0, -0}, extent = {{-108, 88}, {-92, 72}}, rotation = 270)));
  Modelica.Mechanics.Translational.Interfaces.Flange_a axis "1-dim. translational flange that drives the prismatic joint" annotation(Placement(transformation(extent = {{45, 95}, {35, 105}}, origin = {0, 0}, rotation = 0), visible = true, iconTransformation(origin = {0, 0}, extent = {{45, 95}, {35, 105}}, rotation = 0)));
  Modelica.Mechanics.Translational.Interfaces.Flange_b bearing "1-dim. translational flange of the drive bearing of the prismatic joint" annotation(Placement(transformation(extent = {{-35, 95}, {-45, 105}}, origin = {0, 0}, rotation = 0), visible = true, iconTransformation(origin = {0, 0}, extent = {{-35, 95}, {-45, 105}}, rotation = 0)));
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter Boolean showUniversalAxes = true "= true, if universal joint shall be visualized with two cylinders, otherwise with a sphere (provided animation=true)";
  parameter UWBody.Types.Axis n1_a = {0, 0, 1} "Axis 1 of universal joint resolved in frame_a (axis 2 is orthogonal to axis 1 and to line from universal to spherical joint)" annotation(Evaluate = true);
  parameter SI.Position nAxis_ia[3] = {1, 0, 0} "Axis vector along line from origin of frame_a to origin of frame_b, resolved in frame_ia" annotation(Evaluate = true);
  parameter SI.Position s_offset = 0 "Relative distance offset (distance between frame_a and frame_b = s(t) + s_offset)";
  parameter SI.Diameter sphereDiameter = world.defaultJointLength "Diameter of spheres representing the spherical joints" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input Types.Color sphereColor = UWBody.Types.Defaults.JointColor "Color of spheres representing the spherical joints" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Diameter axisDiameter = sphereDiameter / Types.Defaults.JointRodDiameterFraction "Diameter of cylinder on the connecting line from frame_a to frame_b" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input Types.Color axisColor = UWBody.Types.Defaults.SensorColor "Color of cylinder on the connecting line from frame_a to frame_b" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Distance cylinderLength = world.defaultJointLength "Length of cylinders representing the two universal joint axes" annotation(Dialog(tab = "Animation", group = "if animation = true and showUniversalAxes", enable = animation and showUniversalAxes));
  parameter SI.Distance cylinderDiameter = world.defaultJointWidth "Diameter of cylinders representing the two universal joint axes" annotation(Dialog(tab = "Animation", group = "if animation = true and showUniversalAxes", enable = animation and showUniversalAxes));
  input Types.Color cylinderColor = UWBody.Types.Defaults.JointColor "Color of cylinders representing the two universal joint axes" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true and showUniversalAxes", enable = animation and showUniversalAxes));
  parameter Boolean checkTotalPower = false "= true, if total power flowing into this component shall be determined (must be zero)" annotation(Dialog(tab = "Advanced"));
  final parameter Real eAxis_ia[3](each final unit = "1") = Modelica.Math.Vectors.normalizeWithAssert(nAxis_ia) "Unit vector from origin of frame_a to origin of frame_b, resolved in frame_ia";
  final parameter Real e2_ia[3](each final unit = "1") = Modelica.Math.Vectors.normalizeWithAssert(cross(n1_a, eAxis_ia)) "Unit vector in direction of second rotation axis of universal joint, resolved in frame_ia";
  final parameter Real e3_ia[3](each final unit = "1") = cross(eAxis_ia, e2_ia) "Unit vector perpendicular to eAxis_ia and e2_ia, resolved in frame_ia";
  SI.Position s "Relative distance between frame_a and frame_b along axis nAxis = s + s_offset";
  SI.Force f "= axis.f (driving force in the axis; = -bearing.f)";
  SI.Length axisLength "Distance between frame_a and frame_b";
  SI.Power totalPower "Total power flowing into this element, if checkTotalPower=true (otherwise dummy)";
protected
  SI.Force f_c_a[3] "frame_ia.f resolved in frame_a";
  SI.Torque t_cd_a[3] "frame_ia.t + frame_ib.t resolved in frame_a";
  SI.Force f_bd_a[3] "frame_b.f + frame_ib.f resolved in frame_a";
  SI.Position rAxis_0[3] "Position vector from origin of frame_a to origin of frame_b resolved in world frame";
  SI.Position rAxis_a[3] "Position vector from origin of frame_a to origin of frame_b resolved in frame_a";
  Real eAxis_a[3](each final unit = "1") "Unit vector in direction of rAxis_a, resolved in frame_a";
  Real e2_a[3](each final unit = "1") "Unit vector in direction of second rotation axis of universal joint, resolved in frame_a";
  Real e3_a[3](each final unit = "1") "Unit vector perpendicular to eAxis_a and e2_a, resolved in frame_a";
  Real n2_a[3](each final unit = "1") "Vector in direction of second rotation axis of universal joint, resolved in frame_a";
  Real length2_n2_a(unit = "1") "Square of length of vector n2_a";
  Real length_n2_a(unit = "1") "Length of vector n2_a";
  Real der_rAxis_a_L[3](each unit = "1/s") "= der(rAxis_a)/axisLength";
  SI.AngularVelocity w_rel_ia1[3];
  Frames.Orientation R_ia1_a;
  Frames.Orientation R_ia2_a;
  Frames.Orientation R_ia_a "Rotation from frame_a to frame_ia";
  // Real T_ia_a[3, 3] "Transformation matrix from frame_a to frame_ia";
  Visualizers.Advanced.Shape axisCylinder(shapeType = "cylinder", color = axisColor, specularCoefficient = specularCoefficient, length = axisLength, width = axisDiameter, height = axisDiameter, lengthDirection = eAxis_ia, widthDirection = e2_ia, r = frame_ia.r_0, R = frame_ia.R) if world.enableAnimation and animation;
  Visualizers.Advanced.Shape sphericalShape_b(shapeType = "sphere", color = sphereColor, specularCoefficient = specularCoefficient, length = sphereDiameter, width = sphereDiameter, height = sphereDiameter, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, r_shape = {-0.5, 0, 0} * sphereDiameter, r = frame_b.r_0, R = frame_b.R) if world.enableAnimation and animation;
  Visualizers.Advanced.Shape sphericalShape_a(shapeType = "sphere", color = sphereColor, specularCoefficient = specularCoefficient, length = sphereDiameter, width = sphereDiameter, height = sphereDiameter, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, r_shape = {-0.5, 0, 0} * sphereDiameter, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
  Visualizers.Advanced.Shape universalShape1(shapeType = "cylinder", color = cylinderColor, specularCoefficient = specularCoefficient, length = cylinderLength, width = cylinderDiameter, height = cylinderDiameter, lengthDirection = n1_a, widthDirection = {0, 1, 0}, r_shape = -n1_a * (cylinderLength / 2), r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation and showUniversalAxes;
  Visualizers.Advanced.Shape universalShape2(shapeType = "cylinder", color = cylinderColor, specularCoefficient = specularCoefficient, length = cylinderLength, width = cylinderDiameter, height = cylinderDiameter, lengthDirection = e2_ia, widthDirection = {0, 1, 0}, r_shape = -e2_ia * (cylinderLength / 2), r = frame_ia.r_0, R = frame_ia.R) if world.enableAnimation and animation and showUniversalAxes;
equation
  Connections.branch(frame_a.R, frame_ia.R);
  Connections.branch(frame_ia.R, frame_ib.R);
  // Translational flanges
  axisLength = s + s_offset;
  bearing.s = 0;
  axis.s = s;
  axis.f = f;
  // Position vector rAxis from frame_a to frame_b
  rAxis_0 = frame_b.r_0 - frame_a.r_0;
  rAxis_a = Frames.resolve2(frame_a.R, rAxis_0);
  /* Determine relative Rotation R_rel_c from frame_a to frame_ia
           and absolute rotation of frame_a.R.
        */
  axisLength = sqrt(rAxis_0 * rAxis_0);
  assert(axisLength > 1.0e-15, "
      Distance between frame_a and frame_b of a JointUPS joint
      became zero. This is not allowed. If this occurs during
      initialization, the initial conditions are probably wrong.");
  eAxis_a = rAxis_a / axisLength;
  n2_a = cross(n1_a, eAxis_a);
  length2_n2_a = n2_a * n2_a;
  assert(noEvent(length2_n2_a > 1.e-10), "
      A UWBody.Joints.Assemblies.JointUPS joint (consisting of
      a universal, prismatic and spherical joint) is in the singular
      configuration of the universal joint. This means that axis 1 of
      the universal joint defined via parameter \"n1_a\" is parallel to vector
      \"eAxis_ia\" that is directed from the origin of frame_a to the
      origin of frame_b. You may try to use another \"n1_a\" vector.
      ");
  length_n2_a = sqrt(length2_n2_a);
  e2_a = n2_a / length_n2_a;
  e3_a = cross(eAxis_a, e2_a);
  /* The statements below are an efficient implementation of the
           original equations:
             T_ia_a = [eAxis_ia, e2_ia, e3_ia]*transpose([eAxis_a, e2_a, e3_a]);
             R_ia_a = Frames.from_T(T_ia_a,
                           Frames.TransformationMatrices.angularVelocity2(T_ia_a, der(T_ia_a)));
         To perform this, the rotation is split into two parts:
           R_ia_a : Rotation object from frame_a to frame_ia
           R_ia1_a: Rotation object from frame_a to frame_ia1
                      (frame that is fixed in frame_ia such that x-axis
                      is along the rod axis)
                      T = transpose([eAxis_a, e2_a, e3_a]; w = w_rel_ia1
           R_ia2_a: Fixed rotation object from frame_ia1 to frame_ia
                      T = [eAxis_a, e2_ia, e3_ia]; w = zeros(3)
      
         The difficult part is to compute w_rel_ia1:
            w_rel_ia1 = [  e3_a*der(e2_a);
                          -e3_a*der(eAxis_a);
                           e2_a*der(eAxis_a)]
         der(eAxis_a) is directly given, since eAxis_a is a function
         of translational quantities only.
            der(eAxis_a) = (der(rAxis_a) - eAxis_a*(eAxis_a*der(rAxis_a)))/axisLength
            der(n2_a)    = cross(n1_a, der(eAxis_a))
            der(e2_a)    = (der(n2_a) - e2_a*(e2_a*der(n2_a)))/length_n2_a
         Inserting these equations in w_rel_ia1 results in:
            e3_a*der(eAxis_a) = e3_a*der(rAxis_a)/axisLength       // e3_a*eAxis_a = 0
            e2_a*der(eAxis_a) = e2_a*der(rAxis_a)/axisLength       // e2_a*eAxis_a = 0
            e3_a*der(e2_a)    = e3_a*der(n2_a)/length_n2_a       // e3_a*e2_a = 0
                              = e3_a*cross(n1_a, der(eAxis_a))/length_n2_a
                              = e3_a*cross(n1_a, der(rAxis_a) - eAxis_a*(eAxis_a*der(rAxis_a)))/(length_n2_a*axisLength)
                              = e3_a*cross(n1_a, der(rAxis_a))/(length_n2_a*axisLength)
         Furthermore, we have:
           rAxis_a      = resolve2(frame_a.R, rAxis_0);
           der(rAxis_a) = resolve2(frame_a.R, der(rAxis_0)) - cross(frame_a.R.w, rAxis_a));
      */
  der_rAxis_a_L = (Frames.resolve2(frame_a.R, der(rAxis_0)) - cross(frame_a.R.w, rAxis_a)) / axisLength;
  w_rel_ia1 = {e3_a * cross(n1_a, der_rAxis_a_L) / length_n2_a, -e3_a * der_rAxis_a_L, e2_a * der_rAxis_a_L};
  R_ia1_a = Frames.from_T(transpose([eAxis_a, e2_a, e3_a]), w_rel_ia1);
  R_ia2_a = Frames.from_T([eAxis_ia, e2_ia, e3_ia], zeros(3));
  R_ia_a = Frames.absoluteRotation(R_ia1_a, R_ia2_a);
  /*
        T_ia_a = [eAxis_ia, e2_ia, e3_ia]*transpose([eAxis_a, e2_a, e3_a]);
        R_ia_a = Frames.from_T(T_ia_a, Frames.TransformationMatrices.angularVelocity2
          (T_ia_a, der(T_ia_a)));
      */
  // Compute kinematic quantities of frame_ia and frame_ib
  frame_ia.r_0 = frame_a.r_0;
  frame_ib.r_0 = frame_b.r_0;
  frame_ia.R = Frames.absoluteRotation(frame_a.R, R_ia_a);
  frame_ib.R = frame_ia.R;
  /* In the following formulas f_a, f_b, f_ia, f_ib, t_a, t_b, t_ia, t_ib are
           the forces and torques at frame_a, frame_b, frame_ia, frame_ib respectively,
           resolved in frame_a. eAxis, e2, e3 are the unit vectors resolved in frame_a.
           Torque balance at the rod around the origin of frame_a:
             0 = t_a + t_ia + t_ib + cross(rAxis, (f_b+f_ib))
           with
               rAxis = axisLength*eAxis
               f_bd  = f_b + f_ib
               f_bd  = f*eAxis + f_bd[2]*e2 + f_bd[3]*e3
           follows:
               0 = t_a + t_ia + axisLength*(f_bd[2]*e_z - f_bd[3]*e_y)
           The projection of t_a with respect to universal joint axes vanishes:
             e1*t_a = 0
             e2*t_a = 0
           Therefore:
              0 = e1*(t_ia + t_ib) + axisLength*f_bd[2]*(e1*e3)
              0 = e2*(t_ia + t_ib) - axisLength*f_bd[3]
           or
              f_bd = f*eAxis - e2*(e1*(t_ia+t_ib))/(axisLength*(e1*e3)) +
                      e3*(e2*(t_ia+t_ib))/axisLength
           Force balance:
              0 = f_a + f_bd + f_ia
        */
  f_c_a = Frames.resolve1(R_ia_a, frame_ia.f);
  t_cd_a = Frames.resolve1(R_ia_a, frame_ia.t + frame_ib.t);
  f_bd_a = (-eAxis_a * f) - e2_a * (n1_a * t_cd_a / (axisLength * (n1_a * e3_a))) + e3_a * (e2_a * t_cd_a / axisLength);
  zeros(3) = frame_b.f + Frames.resolveRelative(frame_ib.f, frame_ib.R, frame_b.R) - Frames.resolveRelative(f_bd_a, frame_a.R, frame_b.R);
  zeros(3) = frame_b.t;
  zeros(3) = frame_a.f + f_c_a + f_bd_a;
  zeros(3) = frame_a.t + t_cd_a + cross(rAxis_a, f_bd_a);
  // Measure power for test purposes
  if checkTotalPower then
    totalPower = frame_a.f * Frames.resolve2(frame_a.R, der(frame_a.r_0)) + frame_b.f * Frames.resolve2(frame_b.R, der(frame_b.r_0)) + frame_ia.f * Frames.resolve2(frame_ia.R, der(frame_ia.r_0)) + frame_ib.f * Frames.resolve2(frame_ib.R, der(frame_ib.r_0)) + frame_a.t * Frames.angularVelocity2(frame_a.R) + frame_b.t * Frames.angularVelocity2(frame_b.R) + frame_ia.t * Frames.angularVelocity2(frame_ia.R) + frame_ib.t * Frames.angularVelocity2(frame_ib.R) + axis.f * der(axis.s) + bearing.f * der(bearing.s);
  else
    totalPower = 0;
  end if;
  annotation(Documentation(info = "<html>
<p>
This component consists of a <b>universal</b> joint at frame_a,
a <b>spherical</b> joint at frame_b and a <b>prismatic</b> joint along the
line connecting the origin of frame_a and the origin of frame_b,
see the default animation in the following figure (the axes vectors
are not part of the default animation):
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/JointUPS.png\" ALT=\"model Joints.Assemblies.JointUPS\">
</p>

<p>
This joint aggregation has no mass and no inertia and
introduces neither constraints nor potential state variables.
It is especially useful to build up more complicated force elements
where the mass and/or inertia of the force element shall be taken
into account.
</p>
<p>
The universal joint is defined in the following way:
</p>
<ul>
<li> The rotation <b>axis</b> of revolute joint <b>1</b> is along parameter
     vector n1_a which is fixed in frame_a.</li>
<li> The rotation <b>axis</b> of revolute joint <b>2</b> is perpendicular to
     axis 1 and to the line connecting the universal and the spherical joint.</li>
</ul>
<p>
The definition of axis 2 of the universal joint is performed according
to the most often occurring case. In a future release, axis 2 might
be explicitly definable via a parameter. However, the treatment is much more
complicated and the number of operations is considerably higher,
if axis 2 is not orthogonal to axis 1 and to the connecting rod.
</p>
<p>
Note, there is a <b>singularity</b> when axis 1 and the connecting line are parallel
to each other. Therefore, if possible n1_a should be selected in such a way that it
is perpendicular to nAxis_ia in the initial configuration (i.e., the
distance to the singularity is as large as possible).
</p>
<p>
An additional <b>frame_ia</b> is present. It is <b>fixed</b> on the line
connecting the universal and the spherical joint at the
origin of <b>frame_a</b>. The placement of frame_ia on this line
is implicitly defined by the universal joint (frame_a and frame_ia coincide
when the angles of the two revolute joints of the universal joint are zero)
and by parameter vector <b>nAxis_ia</b>, an axis vector directed
along the line from the origin of frame_a to the spherical joint,
resolved in frame_<b>ia</b>.
</p>
<p>
An additional <b>frame_ib</b> is present. It is <b>fixed</b> in the line
connecting the prismatic and the spherical joint at the
origin of <b>frame_b</b>.
It is always parallel to <b>frame_ia</b>.
</p>
<p>
Note, this joint aggregation can be used in cases where
in reality a rod with spherical joints at each end are present.
Such a system has an additional degree of freedom to rotate
the rod along its axis. In practice this rotation is usually
of no interest and is mathematically removed by replacing one
of the spherical joints by a universal joint.
</p>
<p>
The easiest way to define the parameters of this joint is by moving the
MultiBody system in a <b>reference configuration</b> where <b>all frames</b>
of all components are <b>parallel</b> to each other (alternatively,
at least frame_a, frame_ia and frame_ib of the JointUSP joint
should be parallel to each other when defining an instance of this
component).
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.2, grid = {5, 5}), graphics = {Line(visible = true, points = {{-40, 0}, {-40, 90}, {-80, 90}, {-80, 100}}, color = {64, 64, 64}, thickness = 0.5), Text(visible = true, textColor = {64, 64, 64}, extent = {{-140, -90}, {140, -50}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-98, 65}, {-60, 85}}, textString = "ia"), Text(visible = true, textColor = {64, 64, 64}, extent = {{61, 64}, {109, 85}}, textString = "ib"), Line(visible = true, points = {{60, -1}, {60, 90}, {80, 90}, {80, 100}}, color = {64, 64, 64}, thickness = 0.5), Line(visible = true, points = {{60, 90}, {40, 90}, {40, 100}}, color = {64, 64, 64}, thickness = 0.5), Line(visible = true, points = {{-30, 70}, {10, 70}}), Polygon(visible = true, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{30, 70}, {10, 76}, {10, 63}, {30, 70}}), Line(visible = true, points = {{-40, 90}, {-40, 90}, {-40, 100}}, color = {64, 64, 64}, thickness = 0.5), Rectangle(visible = true, origin = {-0.5, 0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-49.5, -5}, {49.5, 5}}), Ellipse(visible = true, origin = {5, 0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{40, -15}, {70, 15}}), Polygon(visible = true, origin = {5, 0}, rotation = 180, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-40.019, 25.992}, {-40.019, 25.992}, {-47.214, 28.972}, {-55, 30}, {-62.786, 28.972}, {-69.981, 25.992}, {-76.213, 21.213}, {-80.992, 14.981}, {-83.972, 7.786}, {-85, 0}, {-83.972, -7.786}, {-80.992, -14.981}, {-76.213, -21.213}, {-69.981, -25.992}, {-62.786, -28.972}, {-55, -30}, {-47.214, -28.972}, {-40.019, -25.992}, {-40.019, -25.992}, {-39.693, -36.955}, {-39.693, -36.955}, {-47.16, -39.224}, {-55, -40}, {-62.84, -39.224}, {-70.307, -36.955}, {-77.192, -33.279}, {-83.284, -28.284}, {-88.279, -22.192}, {-91.955, -15.307}, {-94.224, -7.84}, {-95, 0}, {-94.224, 7.84}, {-91.955, 15.307}, {-88.279, 22.192}, {-83.284, 28.284}, {-77.192, 33.279}, {-70.307, 36.955}, {-62.84, 39.224}, {-55, 40}, {-47.16, 39.224}, {-39.693, 36.955}, {-39.693, 36.955}}, smooth = Smooth.Bezier), Polygon(visible = true, origin = {-60.237, 0}, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-16, 18.667}, {-18.667, 16}, {-18.667, -16}, {-16, -18.667}, {16, -18.667}, {18.667, -16}, {18.667, 16}, {16, 18.667}}), Polygon(visible = true, origin = {-60.237, 0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, points = {{13.333, 34.667}, {-24, 34.667}, {-34.667, 24}, {-34.667, 10.667}, {-37.333, 8}, {-37.333, -8}, {-34.667, -10.667}, {-34.667, -24}, {-24, -34.667}, {13.333, -34.667}, {13.333, -24}, {-18.667, -24}, {-24, -18.667}, {-24, 18.667}, {-18.667, 24}, {13.333, 24}}), Polygon(visible = true, origin = {-60.237, 0}, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{-13.333, 13.333}, {20.237, 13.333}, {25.237, 8}, {30.237, 8}, {30.237, -8}, {25.237, -8}, {20.237, -13.333}, {-13.333, -13.333}}), Ellipse(visible = true, origin = {-60.237, 0}, rotation = 45, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-6, -6}, {6, 6}}), Polygon(visible = true, origin = {-60.237, 0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.VerticalCylinder, points = {{-5.333, -24}, {-5.333, -18.667}, {5.333, -18.667}, {5.333, -24}}), Polygon(visible = true, origin = {-60.237, 0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.VerticalCylinder, points = {{-5.333, 18.667}, {-5.333, 24}, {5.333, 24}, {5.333, 18.667}}), Rectangle(visible = true, origin = {10, 0}, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, extent = {{-5, -20}, {5, 20}}), Rectangle(visible = true, origin = {22.5, 0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, extent = {{-12.5, -10}, {12.5, 10}}), Rectangle(visible = true, origin = {-7.5, 0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, extent = {{-12.5, -20}, {12.5, 20}})}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.2), graphics = {Line(points = {{-60, -70}, {46, -70}}, color = {0, 0, 255}), Polygon(points = {{60, -70}, {45, -64}, {45, -76}, {60, -70}}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-62, -73}, {65, -90}}, textString = "rAxis", lineColor = {0, 0, 255}), Ellipse(extent = {{-100, -40}, {-19, 40}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.Sphere, fillColor = {192, 192, 192}), Ellipse(extent = {{-90, -30}, {-29, 29}}, lineColor = {160, 160, 164}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-60, 41}, {-19, -41}}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Line(points = {{-60, 40}, {-60, -40}}, thickness = 0.5), Ellipse(extent = {{-83, -17}, {-34, 21}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.Sphere, fillColor = {192, 192, 192}), Ellipse(extent = {{-74, -12}, {-40, 15}}, lineColor = {160, 160, 164}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Polygon(points = {{-72, -20}, {-89, 3}, {-69, 25}, {-45, 27}, {-72, -20}}, pattern = LinePattern.None, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineColor = {0, 0, 255}), Line(points = {{-60, 40}, {-60, -10}}, thickness = 0.5), Line(points = {{-49, 20}, {-69, -15}}, thickness = 0.5), Line(points = {{-40, 0}, {-40, 90}, {-80, 90}, {-80, 99}}, color = {95, 95, 95}, thickness = 0.5), Polygon(points = {{7, -1}, {-5, 2}, {-5, -4}, {7, -1}}, lineColor = {0, 0, 255}), Line(points = {{-50, 19}, {-30, 57}}, color = {0, 0, 255}), Text(extent = {{-24, 74}, {7, 53}}, lineColor = {0, 0, 0}, textString = "e2"), Polygon(points = {{-25, 64}, {-33, 56}, {-27, 53}, {-25, 64}}, lineColor = {0, 0, 255}), Line(points = {{-60, 41}, {-60, 65}}, color = {0, 0, 255}), Polygon(points = {{-60, 75}, {-64, 63}, {-56, 63}, {-60, 75}}, lineColor = {0, 0, 255}), Text(extent = {{-96, 82}, {-65, 61}}, lineColor = {0, 0, 0}, textString = "n1"), Line(points = {{-60, -40}, {-60, -72}}, color = {0, 0, 255}), Ellipse(extent = {{20, -40}, {100, 40}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.Sphere, fillColor = {192, 192, 192}), Ellipse(extent = {{30, -30}, {90, 30}}, lineColor = {192, 192, 192}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-22, 45}, {40, -43}}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Ellipse(extent = {{45, 14}, {74, -14}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.Sphere, fillColor = {128, 128, 128}), Line(points = {{60, 0}, {60, -74}}, color = {0, 0, 255}), Rectangle(extent = {{-35, 14}, {-6, 18}}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid, lineColor = {0, 0, 255}), Rectangle(extent = {{-35, -13}, {-6, 14}}, pattern = LinePattern.None, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, lineColor = {0, 0, 255}), Rectangle(extent = {{-6, 6}, {46, 10}}, pattern = LinePattern.None, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid, lineColor = {0, 0, 255}), Rectangle(extent = {{-6, -7}, {46, 6}}, pattern = LinePattern.None, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, lineColor = {0, 0, 255}), Line(points = {{-6, -13}, {-6, 18}}), Text(extent = {{-40, -2}, {-1, -16}}, lineColor = {0, 0, 0}, textString = "nAxis"), Line(points = {{-61, 1}, {-2, 1}}, color = {0, 0, 255}), Polygon(points = {{10, 1}, {-2, 4}, {-2, -2}, {10, 1}}, lineColor = {0, 0, 255}), Line(points = {{60, -1}, {60, 90}, {80, 90}, {80, 99}}, color = {95, 95, 95}, thickness = 0.5), Text(extent = {{-24, 117}, {-9, 102}}, textString = "f", lineColor = {0, 0, 255}), Polygon(points = {{-26, 103}, {-36, 100}, {-26, 97}, {-26, 103}}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid), Polygon(points = {{26, 103}, {36, 100}, {26, 97}, {26, 103}}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid), Line(points = {{14, 100}, {36, 100}}, color = {0, 0, 255}), Text(extent = {{12, 116}, {27, 101}}, textString = "f", lineColor = {0, 0, 255}), Polygon(points = {{30, 93}, {40, 90}, {30, 87}, {30, 93}}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid), Line(points = {{-40, 90}, {40, 90}}, color = {128, 128, 128}), Line(points = {{-25, 100}, {-10, 100}}, color = {0, 0, 255}), Text(extent = {{-18, 90}, {19, 77}}, lineColor = {128, 128, 128}, textString = "s"), Line(points = {{60, 90}, {40, 90}, {40, 98}}, color = {95, 95, 95}, thickness = 0.5), Line(points = {{-40, 90}, {-40, 96}, {-40, 98}}, color = {135, 135, 135}, thickness = 0.5)}));
end JointUPS;
