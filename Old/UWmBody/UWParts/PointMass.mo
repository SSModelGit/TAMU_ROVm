within UWmBody.UWParts;

model PointMass "Rigid body where body rotation and inertia tensor is neglected (6 potential states)"
  import UWmBody.UWTypes;
  UWInterfaces.Frame_a frame_a "Coordinate system fixed at center of mass point" annotation(Placement(transformation(extent = {{-16, -16}, {16, 16}})));
  parameter Boolean animation = true "= true, if animation shall be enabled (show sphere)";
  parameter SI.Mass m(min = 0) "Mass of mass point";
  input SI.Diameter sphereDiameter = world.defaultBodyDiameter "Diameter of sphere" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.Color sphereColor = UWmBody.UWTypes.Defaults.BodyColor "Color of sphere" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter StateSelect stateSelect = StateSelect.avoid "Priority to use frame_a.r_0, v_0 (= der(frame_a.r_0)) as states" annotation(Dialog(tab = "Advanced"));
  SI.Position r_0[3](start = {0, 0, 0}, each stateSelect = stateSelect) "Position vector from origin of world frame to origin of frame_a, resolved in world frame" annotation(Dialog(group = "Initialization", showStartAttribute = true));
  Modelica.SIunits.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = stateSelect) "Absolute velocity of frame_a, resolved in world frame (= der(r_0))" annotation(Dialog(group = "Initialization", showStartAttribute = true));
  Modelica.SIunits.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_a resolved in world frame (= der(v_0))" annotation(Dialog(group = "Initialization", showStartAttribute = true));
protected
  outer UWmBody.UWWorld world;
  // Declarations for animation
  UWVisualizers.Advanced.Shape sphere(shapeType = "sphere", color = sphereColor, specularCoefficient = specularCoefficient, length = sphereDiameter, width = sphereDiameter, height = sphereDiameter, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, r_shape = -{1, 0, 0} * sphereDiameter / 2, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
equation
  // If any possible, do not use the connector as root
  Connections.potentialRoot(frame_a.R, 10000);
  if Connections.isRoot(frame_a.R) then
    assert(cardinality(frame_a) == 0, "
      A UWmBody.UWParts.PointMass model is connected in
      a way, so that no equations are present to compute frame_a.R
      (the orientation object in the connector). Setting frame_a.R to
      an arbitrary value in the PointMass model, might lead to a wrong
      overall model, depending on how the PointMass model is used.
         You can avoid this message, by providing equations that
      compute the orientation object, e.g., by using the
      UWmBody.UWJoints.FreeMotion joint.
         If a PointMass model is not connected at all, the
      orientation object is set to a unit rotation. But this is
      the only case where this is done.
      ");
    frame_a.R = Frames.nullRotation();
  else
    frame_a.t = zeros(3);
  end if;
  // Newton equation: f = m*(a-g)
  r_0 = frame_a.r_0;
  v_0 = der(r_0);
  a_0 = der(v_0);
  frame_a.f = m * Frames.resolve2(frame_a.R, a_0 - world.gravityAcceleration(r_0));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -90}, {150, -60}}, textString = "m=%m"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 60}, {150, 100}}, textString = "%name"), Ellipse(visible = true, lineColor = {10, 90, 224}, fillColor = {156, 203, 255}, fillPattern = FillPattern.Sphere, extent = {{-50, -50}, {50, 50}})}), Documentation(info = "<html>
<p>
<b>Rigid body</b> where the inertia tensor is neglected.
This body is
solely defined by its mass.
By default, this component is visualized by a <b>sphere</b> that has
its center at frame_a. Note, that
the animation may be switched off via parameter animation = <b>false</b>.
</p>

<p>
Every PointMass has potential states. If possible a tool will select
the states of joints and not the states of PointMass because this is
usually the most efficient choice. In this case the position and
velocity of frame_a of the body will be computed
by the component that is connected to frame_a. However, if a PointMass is moving
freely in space, variables of the PointMass have to be used as states. The potential
states are: The <b>position vector</b> frame_a.r_0 from the origin of the
world frame to the origin of frame_a of the body, resolved in
the world frame and the <b>absolute velocity</b> v_0 of the origin of
frame_a, resolved in the world frame (= der(frame_a.r_0)).
</p>

<p>
Whether or not variables of the body are used as states is usually
automatically selected by the Modelica translator. If parameter
<b>enforceStates</b> is set to <b>true</b> in the \"Advanced\" menu,
then PointMass variables frame_a.r_0 and der(frame_a.r_0)
are forced to be used as states.
</p>
</html>"));
end PointMass;
