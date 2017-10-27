package UBody
  extends Modelica.Icons.Package;

  model WBody
    /* extends Modelica.Mechanics.MultiBody.Parts.Body;
  parameter Modelica.SIunits.Density density = 7700 "Density of object";
  Modelica.Mechanics.MultiBody.Interfaces.Frame_resolve frame_w annotation(Placement(visible = true, transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.SIunits.Acceleration b_a[3] "Buoyant acceleration within fluid field";
protected
  outer WaterF waterF;
equation
  b_a = waterF.bA(d = density);
  frame_w.f = m*b_a; */
    import SI = Modelica.SIunits;
    import Types = Modelica.Mechanics.MultiBody.Types;
    import Cv = Modelica.SIunits.Conversions;
    import C = Modelica.Constants;
    Modelica.Mechanics.MultiBody.Parts.Body body(r_CM = r_CM, m = m, r_0.start = r_0_start, v_0.start = v_0_start, I_11 = I_11, I_22 = I_22, I_33 = I_33, I_21 = I_21, I_31 = I_31, I_32 = I_32, animation = animation, a_0.start = a_0_start, angles_fixed = angles_fixed, angles_start = angles_start, sequence_start = sequence_start, w_0_fixed = w_0_fixed, w_0_start = w_0_start, z_0_fixed = z_0_fixed, z_0_start = z_0_start, sphereDiameter = sphereDiameter, sphereColor = sphereColor, cylinderDiameter = cylinderDiameter, cylinderColor = cylinderColor, specularCoefficient = specularCoefficient, enforceStates = enforceStates, useQuaternions = useQuaternions, sequence_angleStates = sequence_angleStates, r_0.fixed = r_0_fixed, v_0.fixed = v_0_fixed, a_0.fixed = a_0_fixed) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    parameter Modelica.SIunits.Density density = 7700 "Density of object";
    parameter Modelica.SIunits.DimensionlessRatio c_d annotation(Dialog);
    parameter Modelica.SIunits.Area a_wb annotation(Dialog);
    SI.Force f_d[3];
    Modelica.SIunits.Acceleration b_a[3] "Buoyant acceleration within fluid field";
    Modelica.Mechanics.MultiBody.Forces.WorldForce force annotation(Placement(visible = true, transformation(origin = {-51.931, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    parameter Boolean r_0_fixed annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
    parameter Boolean v_0_fixed annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
    parameter Boolean a_0_fixed annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
    parameter SI.Position r_0_start[3] annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
    parameter SI.Velocity v_0_start[3] annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
    parameter SI.Acceleration a_0_start[3] annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
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
    parameter Types.RotationSequence sequence_start = {1, 2, 3} "Sequence of rotations to rotate frame_a into frame_b at initial time" annotation(Evaluate = true, Dialog(tab = "Initialization"));
    parameter Boolean w_0_fixed = false "= true, if w_0_start are used as initial values, else as guess values" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
    parameter SI.AngularVelocity w_0_start[3] = {0, 0, 0} "Initial or guess values of angular velocity of frame_a resolved in world frame" annotation(Dialog(tab = "Initialization"));
    parameter Boolean z_0_fixed = false "= true, if z_0_start are used as initial values, else as guess values" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Initialization"));
    parameter SI.AngularAcceleration z_0_start[3] = {0, 0, 0} "Initial values of angular acceleration z_0 = der(w_0)" annotation(Dialog(tab = "Initialization"));
    parameter SI.Diameter sphereDiameter = world.defaultBodyDiameter "Diameter of sphere" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
    input Types.Color sphereColor = Modelica.Mechanics.MultiBody.Types.Defaults.BodyColor "Color of sphere" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
    parameter SI.Diameter cylinderDiameter = sphereDiameter / Types.Defaults.BodyCylinderDiameterFraction "Diameter of cylinder" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
    input Types.Color cylinderColor = sphereColor "Color of cylinder" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
    input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
    parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(Evaluate = true, Dialog(tab = "Advanced"));
    parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(Evaluate = true, Dialog(tab = "Advanced"));
    parameter Types.RotationSequence sequence_angleStates = {1, 2, 3} "Sequence of rotations to rotate world frame into frame_a around the 3 angles used as potential states" annotation(Evaluate = true, Dialog(tab = "Advanced", enable = not useQuaternions));
  protected
    outer WaterF waterF;
    outer Modelica.Mechanics.MultiBody.World world;
  equation
    b_a = waterF.bA(d = density);
    f_d = waterF.fld(v = body.v_0, c = c_d, A = a_wb);
    force.force = body.m * b_a + f_d;
  equation
    connect(frame_a, body.frame_a) annotation(Line(visible = true, origin = {-55, 0}, points = {{-45, 0}, {45, 0}}, color = {95, 95, 95}));
    connect(force.frame_b, body.frame_a) annotation(Line(visible = true, origin = {-19.489, 30}, points = {{-22.442, 30}, {6.476, 30}, {6.476, -30}, {9.489, -30}}, color = {95, 95, 95}));
  end WBody;

  model WaterF "World coordinate system + gravity field + default animation definition"
    parameter Modelica.SIunits.Density pho_field = 1000 "Density of fluid field" annotation(Dialog);
    replaceable function bA = UBody.FunkyWater(g = world.g * Modelica.Math.Vectors.normalizeWithAssert(world.n), pho_fluid = pho_field) constrainedby UBody.FunkWater;
    replaceable function fld = UBody.HeavyWater(pho_fluid = pho_field) constrainedby UBody.FatWater;
  protected
    outer Modelica.Mechanics.MultiBody.World world;
    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, origin = {-0.368, 1.472}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid, extent = {{-96.446, -93.87}, {96.446, 93.87}})}), defaultComponentName = "waterF", defaultComponentPrefixes = "inner", missingInnerMessage = "No \"waterF\" component is defined.");
  end WaterF;

  function FunkyWater
    extends Modelica.Icons.Function;
    extends UBody.FunkWater;
    input Modelica.SIunits.Density pho_fluid "Density of fluid surrounding submerged object";
    input Modelica.SIunits.Acceleration g[3] "Constant gravity acceleration value, resolved in world frame, opposite to direction of actual gravity" annotation(Dialog);
  protected
    final parameter Modelica.SIunits.RelativeDensity d_star = pho_fluid / d;
  algorithm
    buoyant_a := -d_star * g;
    annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end FunkyWater;

  partial function FunkWater
    extends Modelica.Icons.Function;
    input Modelica.SIunits.Density d "Density of submerged object";
    output Modelica.SIunits.Acceleration buoyant_a[3] "Buoyant acceleration experienced by submerged object from surrounding fluid";
  end FunkWater;

  partial function FatWater
    extends Modelica.Icons.Function;
    input Modelica.SIunits.Velocity v[:];
    output Modelica.SIunits.Force f_d[:];
    annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  end FatWater;

  function HeavyWater
    extends Modelica.Icons.Function;
    extends UBody.FatWater;
    input Modelica.SIunits.DimensionlessRatio c annotation(Dialog);
    input Modelica.SIunits.Area A annotation(Dialog);
    input Modelica.SIunits.Density pho_fluid annotation(Dialog);
  algorithm
    // f_d := -0.5 * pho_fluid * c * A * Modelica.Math.Vectors.length(v) * v * 0.01;
    f_d := -0.5 * pho_fluid * c * A * v;
  end HeavyWater;
end UBody;
