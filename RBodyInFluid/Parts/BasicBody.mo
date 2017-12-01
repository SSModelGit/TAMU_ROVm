within RBodyInFluid.Parts;

model BasicBody
  Modelica.Mechanics.MultiBody.Parts.Body body(r_CM = r_CM, m = m, I_11 = I_11, I_22 = I_22, I_33 = I_33, I_21 = I_21, I_31 = I_31, I_32 = I_32, animation = animation, angles_fixed = angles_fixed, angles_start = angles_start, sequence_start = sequence_start, w_0_fixed = w_0_fixed, w_0_start = w_0_start, z_0_fixed = z_0_fixed, z_0_start = z_0_start, sphereDiameter = sphereDiameter, sphereColor = sphereColor, cylinderDiameter = cylinderDiameter, cylinderColor = cylinderColor, specularCoefficient = specularCoefficient, enforceStates = enforceStates, useQuaternions = useQuaternions, sequence_angleStates = sequence_angleStates) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  parameter Modelica.SIunits.Density density = 7700 "Density of object";
  parameter Modelica.SIunits.DimensionlessRatio c_d "Constant of viscosity/something" annotation(Dialog);
  parameter Modelica.SIunits.Area A "Cross sectional area of the object" annotation(Dialog);
  SI.Force f_d[3] "Drag force";
  Modelica.SIunits.Acceleration b_f[3] "Buoyant acceleration within fluid field";
  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque field(animation = false) "Overall external force due to fields" annotation(Placement(visible = true, transformation(origin = {-51.931, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Parameters for the Body object
  SI.Position r_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Position vector from origin of world frame to origin of frame_a" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Absolute velocity of frame_a, resolved in world frame (= der(r_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_a resolved in world frame (= der(v_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  parameter Boolean animation = true "= true, if animation shall be enabled (show cylinder and sphere)";
  parameter SI.Position r_CM[3](start = {0, 0, 0}) "Vector from frame_a to center of mass, resolved in frame_a";
  parameter SI.Mass m(min = 0, start = 1) "Mass of rigid body";
  parameter SI.Inertia I_11(min = 0) = 0.001 "(1,1) element of inertia tensor" annotation(Dialog(group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
  parameter SI.Inertia I_22(min = 0) = 0.001 "(2,2) element of inertia tensor" annotation(Dialog(group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
  parameter SI.Inertia I_33(min = 0) = 0.001 "(3,3) element of inertia tensor" annotation(Dialog(group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
  parameter SI.Inertia I_21(min = -C.inf) = 0 "(2,1) element of inertia tensor" annotation(Dialog(group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
  parameter SI.Inertia I_31(min = -C.inf) = 0 "(3,1) element of inertia tensor" annotation(Dialog(group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
  parameter SI.Inertia I_32(min = -C.inf) = 0 "(3,2) element of inertia tensor" annotation(Dialog(group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
  parameter Boolean angles_fixed = false "= true, if angles_start are used as initial values, else as guess values" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
  parameter SI.Angle angles_start[3] = {0, 0, 0} "Initial values of angles to rotate frame_a around 'sequence_start' axes into frame_b" annotation(Dialog(tab = "Initialization"));
  parameter Modelica.Mechanics.MultiBody.Types.RotationSequence sequence_start = {1, 2, 3} "Sequence of rotations to rotate frame_a into frame_b at initial time" annotation(Evaluate = true, Dialog(tab = "Initialization"));
  parameter Boolean w_0_fixed = false "= true, if w_0_start are used as initial values, else as guess values" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
  parameter SI.AngularVelocity w_0_start[3] = {0, 0, 0} "Initial or guess values of angular velocity of frame_a resolved in world frame" annotation(Dialog(tab = "Initialization"));
  parameter Boolean z_0_fixed = false "= true, if z_0_start are used as initial values, else as guess values" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
  parameter SI.AngularAcceleration z_0_start[3] = {0, 0, 0} "Initial values of angular acceleration z_0 = der(w_0)" annotation(Dialog(tab = "Initialization"));
  parameter SI.Diameter sphereDiameter = world.defaultBodyDiameter "Diameter of sphere" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input Modelica.Mechanics.MultiBody.Types.Color sphereColor = Modelica.Mechanics.MultiBody.Types.Defaults.BodyColor "Color of sphere" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Diameter cylinderDiameter = sphereDiameter / Modelica.Mechanics.MultiBody.Types.Defaults.BodyCylinderDiameterFraction "Diameter of cylinder" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input Modelica.Mechanics.MultiBody.Types.Color cylinderColor = sphereColor "Color of cylinder" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  input Modelica.Mechanics.MultiBody.Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter Modelica.Mechanics.MultiBody.Types.RotationSequence sequence_angleStates = {1, 2, 3} "Sequence of rotations to rotate world frame into frame_a around the 3 angles used as potential states" annotation(Evaluate = true, Dialog(tab = "Advanced", enable = not useQuaternions));
protected
  // Fields
  outer RBodyInFluid.Fields.WaterField waterField;
  outer Modelica.Mechanics.MultiBody.World world;
equation
  b_f = waterField.waterBuoyantForce(d = density, m = body.m);
  f_d = waterField.waterDragForce(v = body.v_0, c = c_d, A = A);
  field.force = b_f + f_d;
  field.torque = cross(r_CM, field.force);
  r_0 = frame_a.r_0;
  v_0 = der(r_0);
  a_0 = der(v_0);
  connect(frame_a, body.frame_a) annotation(Line(visible = true, origin = {-55, 0}, points = {{-45, 0}, {45, 0}}, color = {95, 95, 95}));
  connect(field.frame_b, body.frame_a) annotation(Line(visible = true, origin = {-19.489, 30}, points = {{-22.442, 30}, {6.476, 30}, {6.476, -30}, {9.489, -30}}, color = {95, 95, 95}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, fillColor = {0, 0, 100}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Rectangle(visible = true, lineColor = {10, 90, 224}, fillColor = {156, 203, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, -31}, {-3, 30}}, radius = 10), Text(visible = true, textColor = {255, 255, 255}, extent = {{-150, -100}, {150, -70}}, textString = "m=%m"), Text(visible = true, textColor = {255, 255, 255}, extent = {{-150, 70}, {150, 110}}, textString = "%name"), Ellipse(visible = true, lineColor = {10, 90, 224}, fillColor = {156, 203, 255}, fillPattern = FillPattern.Sphere, extent = {{-20, -60}, {100, 60}}), Line(visible = true, origin = {1, -3}, points = {{-56, 78}, {-56, -72}}, color = {255, 255, 255}, pattern = LinePattern.Dash, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Line(visible = true, origin = {-2, -3}, rotation = -180, points = {{2, 78}, {2, -72}}, color = {255, 255, 255}, pattern = LinePattern.Dash, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Line(visible = true, origin = {9, -65}, rotation = -630, points = {{66, 80}, {66, -70}}, color = {255, 255, 255}, pattern = LinePattern.Dash, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30)}));
end BasicBody;
