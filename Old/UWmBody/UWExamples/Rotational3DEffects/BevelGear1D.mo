within UWmBody.UWExamples.Rotational3DEffects;

model BevelGear1D "Demonstrates the usage of a BevelGear1D model and how to calculate the power of such an element"
  import UWmBody.UWFrames;
  extends Modelica.Icons.Example;
  parameter UWmBody.UWTypes.Axis na = {1, 0, 0} "Axis of rotation of left gear axis";
  parameter UWmBody.UWTypes.Axis nb = {0, 1, 0} "Axis of rotation of right gear axis";
  inner UWmBody.UWWorld world(final driveTrainMechanics3D = true) annotation(Placement(transformation(extent = {{-80, -20}, {-60, 0}}, origin = {-20, -20}, rotation = 0), visible = true));
  UWmBody.UWParts.Rotor1D inertia1(J = 1.1, a(fixed = false), phi(fixed = true, start = 0), w(fixed = true, start = 0), n = na) annotation(Placement(transformation(extent = {{-30, 60}, {-10, 80}}, origin = {0, -20}, rotation = 0), visible = true));
  UWmBody.UWParts.Rotor1D inertia2(J = 18.2, n = nb) annotation(Placement(transformation(extent = {{30, 60}, {50, 80}}, origin = {-0, -20}, rotation = 0), visible = true));
  UWmBody.UWParts.BevelGear1D bevelGear(ratio = 10, n_a = na, n_b = nb) annotation(Placement(transformation(extent = {{0, 60}, {20, 80}}, origin = {0, -20}, rotation = 0), visible = true));
  UWmBody.UWJoints.Revolute revolute1(useAxisFlange = true, n = {1, 0, 0}, stateSelect = StateSelect.always, phi(fixed = true), w(fixed = true)) annotation(Placement(transformation(extent = {{-46, 0}, {-26, -20}}, origin = {-14, -20}, rotation = 0), visible = true));
  UWmBody.UWJoints.Revolute revolute2(useAxisFlange = true, n = {0, 1, 0}, stateSelect = StateSelect.always, phi(fixed = true), w(fixed = true)) annotation(Placement(transformation(extent = {{0, 0}, {20, -20}}, origin = {-10, -20}, rotation = 0), visible = true));
  UWmBody.UWJoints.Revolute revolute3(useAxisFlange = true, n = {0, 0, 1}, stateSelect = StateSelect.always, phi(fixed = true), w(fixed = true)) annotation(Placement(transformation(extent = {{46, 0}, {66, -20}}, origin = {-6, -20}, rotation = 0), visible = true));
  Modelica.Mechanics.Rotational.Sources.Torque torque1 annotation(Placement(transformation(extent = {{-52, -45}, {-42, -35}}, origin = {-18, -20}, rotation = 0), visible = true));
  Modelica.Mechanics.Rotational.Sources.Torque torque2 annotation(Placement(transformation(extent = {{-4, -45}, {6, -35}}, origin = {-16, -20}, rotation = 0), visible = true));
  Modelica.Mechanics.Rotational.Sources.Torque torque3 annotation(Placement(transformation(extent = {{42, -45}, {52, -35}}, origin = {-12, -20}, rotation = 0), visible = true));
  Modelica.Blocks.Sources.Sine sine1(amplitude = 110, freqHz = 5) annotation(Placement(transformation(extent = {{-60, -38.333}, {-50, -28.333}}, origin = {-30, -26.667}, rotation = 0), visible = true));
  Modelica.Blocks.Sources.Sine sine2(amplitude = 120, freqHz = 6) annotation(Placement(transformation(extent = {{-20, -38.333}, {-10, -28.333}}, origin = {-20, -26.667}, rotation = 0), visible = true));
  Modelica.Blocks.Sources.Sine sine3(amplitude = 130, freqHz = 7) annotation(Placement(transformation(extent = {{20, -38.333}, {30, -28.333}}, origin = {-10, -26.667}, rotation = 0), visible = true));
  UWmBody.UWParts.Mounting1D mounting1D(n = na) annotation(Placement(transformation(extent = {{-72, 46}, {-52, 66}}, origin = {-8, -36}, rotation = 0), visible = true));
  Modelica.Mechanics.Rotational.Sources.Torque torque(useSupport = true) annotation(Placement(transformation(extent = {{-58, 60}, {-38, 80}}, origin = {-2, -20}, rotation = 0), visible = true));
  Modelica.Blocks.Sources.Sine sine4(amplitude = 140, freqHz = 8) annotation(Placement(transformation(extent = {{-75, 53.333}, {-65, 63.333}}, origin = {-15, -8.333}, rotation = 0), visible = true));
  UWmBody.UWParts.BodyBox bodyBox(r = {0.1, 0.1, 0.1}, length = 0.1, width = 0.1) annotation(Placement(transformation(extent = {{76, -20}, {96, 0}}, origin = {4, -20}, rotation = 0), visible = true));
  UWSensors.AbsoluteAngularVelocity sensor1(resolveInFrame = UWmBody.UWTypes.ResolveInFrameA.frame_a) annotation(Placement(transformation(extent = {{62, 42}, {82, 62}}, origin = {-2, -22}, rotation = 0), visible = true));
  UWSensors.CutTorque sensor2(resolveInFrame = UWmBody.UWTypes.ResolveInFrameA.frame_a, animation = false) annotation(Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = -90, origin = {10, 20}), visible = true));
  Modelica.SIunits.AngularVelocity ws[3] = sensor1.w;
  Modelica.SIunits.Power bevelGearPower;
equation
  bevelGearPower = (ws + der(bevelGear.flange_a.phi) * na) * bevelGear.flange_a.tau * na + (ws + der(bevelGear.flange_b.phi) * nb) * bevelGear.flange_b.tau * nb + ws * sensor2.torque;
  assert(abs(bevelGearPower) < 1e-3, "Error, energy balance of bevel gear is wrong");
  connect(inertia1.flange_b, bevelGear.flange_a) annotation(Line(points = {{-10, 70}, {0, 70}}, visible = true, origin = {0, -20}, color = {64, 64, 64}));
  connect(bevelGear.flange_b, inertia2.flange_a) annotation(Line(points = {{-5, 0}, {5, 0}}, visible = true, origin = {25, 50}, color = {64, 64, 64}));
  connect(world.frame_b, revolute1.frame_a) annotation(Line(points = {{-7.5, 0}, {12.5, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-72.5, -30}));
  connect(revolute1.frame_b, revolute2.frame_a) annotation(Line(points = {{-15, 0}, {15, -0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-25, -30}));
  connect(revolute2.frame_b, revolute3.frame_a) annotation(Line(points = {{-15, -0}, {15, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {25, -30}));
  connect(torque1.flange, revolute1.axis) annotation(Line(points = {{-55, -40}, {-45, -40}, {-45, -20}}, visible = true, origin = {-5, -20}, color = {64, 64, 64}));
  connect(torque2.flange, revolute2.axis) annotation(Line(points = {{-5, -40}, {5, -40}, {5, -20}}, visible = true, origin = {-5, -20}, color = {64, 64, 64}));
  connect(torque3.flange, revolute3.axis) annotation(Line(points = {{45, -40}, {55, -40}, {55, -20}}, visible = true, origin = {-5, -20}, color = {64, 64, 64}));
  connect(sine1.y, torque1.tau) annotation(Line(points = {{-4.25, 0}, {4.25, -0}}, color = {1, 37, 163}, visible = true, origin = {-75.25, -60}));
  connect(torque2.tau, sine2.y) annotation(Line(points = {{4.25, -0}, {-4.25, -0}}, color = {0, 36, 164}, visible = true, origin = {-25.25, -60}));
  connect(torque3.tau, sine3.y) annotation(Line(points = {{4.25, -0}, {-4.25, 0}}, color = {0, 36, 164}, visible = true, origin = {24.75, -60}));
  connect(torque.flange, inertia1.flange_a) annotation(Line(points = {{-5, 0}, {5, 0}}, visible = true, origin = {-35, 50}, color = {64, 64, 64}));
  connect(torque.support, mounting1D.flange_b) annotation(Line(points = {{3.333, 13.333}, {3.333, -6.667}, {-6.667, -6.667}}, visible = true, origin = {-53.333, 26.667}));
  connect(mounting1D.frame_a, revolute3.frame_b) annotation(Line(points = {{-70, 30}, {-70, 20}, {70, 20}, {70, -10}, {60, -10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -20}));
  connect(torque.tau, sine4.y) annotation(Line(points = {{8.75, 0}, {-8.75, 0}}, color = {0, 36, 164}, visible = true, origin = {-70.75, 50}));
  connect(revolute3.frame_b, bodyBox.frame_a) annotation(Line(points = {{-7.5, -0}, {12.5, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {67.5, -30}));
  connect(inertia1.frame_a, revolute3.frame_b) annotation(Line(points = {{-20, 60}, {-20, 40}, {-20, 40}, {-20, 20}, {70, 20}, {70, -10}, {60, -10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -20}));
  connect(inertia2.frame_a, revolute3.frame_b) annotation(Line(points = {{40, 60}, {40, 20}, {70, 20}, {70, -10}, {60, -10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -20}));
  connect(bevelGear.frame_a, sensor1.frame_a) annotation(Line(points = {{10, 60}, {25, 60}, {25, 50}, {60, 50}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -20}));
  connect(bevelGear.frame_a, sensor2.frame_b) annotation(Line(points = {{0, 5}, {0, -5}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, 35}));
  connect(sensor2.frame_a, revolute3.frame_b) annotation(Line(points = {{10, 30}, {10, 20}, {70, 20}, {70, -10}, {60, -10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -20}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})), Documentation(info = "<html>
<p>
This model consists of a drive train with two inertias that are coupled by a bevel gear
(with 90 degree angle between the two gear flanges).
This drive train is mounted on a body that is rotated along three axes.
The drive train is modeled with 1D rotational elements that take into account 3D effects.
</p>

<p>
The bevelGear component consists of two rotational flanges (for the gear flanges) and one 3D frame
(for the support/mounting).
Since the bevelGear does not store energy, the power balance must hold (the total sum
of inflowing and outflowing energy must be zero). One has to be careful, when
computing the energy flow of hybrid 1D/3D component: The angular velocities of rotational flanges
are with respect to the support frame (so the moving body on which the drive train
is mounted). Therefore, when computing the energy flow, first the absolute angular velocities
of the flanges have to be calculated. In this example model, this is performed in the following way
(na and nb are the axes of rotations of the gear flanges, and ws is the
angular velocity of the support frame):
</p>

<pre>
    import Modelica.Mechanics.MultiBody.Frames;
    import SI=Modelica.SIunits;
    SI.Power           bevelGearPower;
    SI.AngularVelocity ws[3] = Frames.angularVelocity2(bevelGear.frame_a.R);
  <b>equation</b>
    bevelGearPower = (ws + der(bevelGear.flange_a.phi)*na)*bevelGear.flange_a.tau*na +
                     (ws + der(bevelGear.flange_b.phi)*nb)*bevelGear.flange_b.tau*nb +
                     ws*bevelGear.frame_a.t;
</pre>
<p>
The total energy flow bevelGearPower must be zero. If a relative tolerance of 1e-4 is used
for simulation, bevelGearPower is in the order of 1e-8 (and smaller for a smaller relative
tolerance).
</p>
</html>"), experiment(StopTime = 1.0));
end BevelGear1D;
