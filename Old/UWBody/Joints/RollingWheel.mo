within UWBody.Joints;

model RollingWheel "Joint (no mass, no inertia) that describes an ideal rolling wheel (rolling on the plane z=0)"
  import UWBody.Frames;
  UWBody.Interfaces.Frame_a frame_a "Frame fixed in wheel center point. x-Axis: upwards, y-axis: along wheel axis" annotation(Placement(transformation(extent = {{-16, -16}, {16, 16}})));
  parameter SI.Radius wheelRadius "Wheel radius";
  parameter StateSelect stateSelect = StateSelect.always "Priority to use generalized coordinates as states" annotation(HideResult = true, Evaluate = true);
  SI.Position x(start = 0, stateSelect = stateSelect) "x-coordinate of wheel axis";
  SI.Position y(start = 0, stateSelect = stateSelect) "y-coordinate of wheel axis";
  SI.Position z;
  Modelica.SIunits.Angle angles[3](start = {0, 0, 0}, each stateSelect = stateSelect) "Angles to rotate world-frame in to frame_a around z-, y-, x-axis" annotation(Dialog(group = "Initialization", showStartAttribute = true));
  SI.AngularVelocity der_angles[3](start = {0, 0, 0}, each stateSelect = stateSelect) "Derivative of angles" annotation(Dialog(group = "Initialization", showStartAttribute = true));
  SI.Position r_road_0[3] "Position vector from world frame to contact point on road, resolved in world frame";
  // Contact force
  SI.Force f_wheel_0[3] "Contact force acting on wheel, resolved in world frame";
  SI.Force f_n "Contact force acting on wheel in normal direction";
  SI.Force f_lat "Contact force acting on wheel in lateral direction";
  SI.Force f_long "Contact force acting on wheel in longitudinal direction";
  SI.Position err "|r_road_0 - frame_a.r_0| - wheelRadius (must be zero; used for checking)";
protected
  Real e_axis_0[3] "Unit vector along wheel axis, resolved in world frame";
  SI.Position delta_0[3](start = {0, 0, -wheelRadius}) "Distance vector from wheel center to contact point";
  // Coordinate system at contact point
  Real e_n_0[3] "Unit vector in normal direction of road at contact point, resolved in world frame";
  Real e_lat_0[3] "Unit vector in lateral direction of wheel at contact point, resolved in world frame";
  Real e_long_0[3] "Unit vector in longitudinal direction of wheel at contact point, resolved in world frame";
  // Road description
  SI.Position s "Road surface parameter 1";
  SI.Position w "Road surface parameter 2";
  Real e_s_0[3] "Road heading at (s,w), resolved in world frame (unit vector)";
  // Slip velocities
  Modelica.SIunits.Velocity v_0[3] "Velocity of wheel center, resolved in world frame";
  SI.AngularVelocity w_0[3] "Angular velocity of wheel, resolved in world frame";
  Modelica.SIunits.Velocity vContact_0[3] "Velocity of wheel contact point, resolved in world frame";
  // Utility vectors
  Real aux[3];
equation
  // frame_a.R is computed from generalized coordinates
  Connections.root(frame_a.R);
  frame_a.r_0 = {x, y, z};
  der_angles = der(angles);
  frame_a.R = Frames.axesRotations({3, 2, 1}, angles, der_angles);
  // Road description
  r_road_0 = {s, w, 0};
  e_n_0 = {0, 0, 1};
  e_s_0 = {1, 0, 0};
  // Coordinate system at contact point (e_long_0, e_lat_0, e_n_0)
  e_axis_0 = Frames.resolve1(frame_a.R, {0, 1, 0});
  aux = cross(e_n_0, e_axis_0);
  e_long_0 = aux / Modelica.Math.Vectors.length(aux);
  e_lat_0 = cross(e_long_0, e_n_0);
  // Determine point on road where the wheel is in contact with the road
  delta_0 = r_road_0 - frame_a.r_0;
  0 = delta_0 * e_axis_0;
  0 = delta_0 * e_long_0;
  // One holonomic positional constraint equation (no penetration in to the ground)
  0 = wheelRadius - delta_0 * cross(e_long_0, e_axis_0);
  // only for testing
  err = Modelica.Math.Vectors.length(delta_0) - wheelRadius;
  // Slip velocities
  v_0 = der(frame_a.r_0);
  w_0 = Frames.angularVelocity1(frame_a.R);
  vContact_0 = v_0 + cross(w_0, delta_0);
  // Two non-holonomic constraint equations on velocity level (ideal rolling, no slippage)
  0 = vContact_0 * e_long_0;
  0 = vContact_0 * e_lat_0;
  // Contact force
  f_wheel_0 = f_n * e_n_0 + f_lat * e_lat_0 + f_long * e_long_0;
  // Force and torque balance at the wheel center
  zeros(3) = frame_a.f + Frames.resolve2(frame_a.R, f_wheel_0);
  zeros(3) = frame_a.t + Frames.resolve2(frame_a.R, cross(delta_0, f_wheel_0));
  // Guard against singularity
  assert(abs(e_n_0 * e_axis_0) < 0.99, "Wheel lays nearly on the ground (which is a singularity)");
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, fillColor = {175, 175, 175}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, -80}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-154, 84}, {146, 124}}, textString = "%name"), Ellipse(visible = true, rotation = 45, lineColor = {128, 128, 128}, fillColor = {230, 230, 230}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-80, -80}, {80, 80}}), Ellipse(visible = true, rotation = 45, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, extent = {{-80, -80}, {80, 80}}), Ellipse(visible = true, rotation = 45, lineColor = {128, 128, 128}, fillColor = {230, 230, 230}, pattern = LinePattern.None, fillPattern = FillPattern.HorizontalCylinder, extent = {{-50, -50}, {50, 50}})}));
end RollingWheel;
