within UWBody.Examples.Rotational3DEffects;

model MovingActuatedDrive "Demonstrates usage of model Rotor1D mounted on a moving body"
  extends Modelica.Icons.Example;
  Parts.BodyShape bodyCylinder(r = {0.5, 0, 0}, m = 0, I_11 = 2, I_22 = 0, I_33 = 0, shapeType = "cylinder", width = 0.1, animateSphere = false, r_shape = {0.1, 0, 0}, r_CM = {0, 0, 0}) annotation(Placement(transformation(extent = {{50, 10}, {70, 30}})));
  Joints.Revolute revolute(n = {1, 0, 0}, a(fixed = false), phi(fixed = true), w(fixed = true)) annotation(Placement(transformation(extent = {{16, 10}, {36, 30}}, origin = {4, 0}, rotation = 0), visible = true));
  inner World world(g = 0, driveTrainMechanics3D = true) annotation(Placement(transformation(extent = {{-84, 10}, {-64, 30}}, origin = {4, 0}, rotation = 0), visible = true));
  Forces.Torque torque annotation(Placement(transformation(extent = {{50, 40}, {70, 60}})));
  Modelica.Blocks.Sources.Sine sine1[3](amplitude = {1, 0, 0}, freqHz = {1, 1, 1}) annotation(Placement(transformation(extent = {{16, 70}, {36, 90}}, origin = {4, 0}, rotation = 0), visible = true));
  Parts.Rotor1D rotor1D(J = 2, phi(fixed = true), w(fixed = true)) annotation(Placement(transformation(extent = {{50, -36}, {70, -16}}, origin = {-0, -4}, rotation = 0), visible = true));
  Rotational.Sources.Torque torque1(useSupport = true) annotation(Placement(transformation(extent = {{18, -36}, {38, -16}}, origin = {2, -4}, rotation = 0), visible = true));
  Parts.Mounting1D mounting1D annotation(Placement(transformation(extent = {{4, -56}, {24, -36}}, origin = {-4, -4}, rotation = 0), visible = true));
  Joints.Revolute r1(useAxisFlange = true, n = {0, 1, 0}) annotation(Placement(transformation(extent = {{-32, 10}, {-12, 30}}, origin = {12, 0}, rotation = 0), visible = true));
  Rotational.Sources.Position position1(useSupport = true, w(fixed = true)) annotation(Placement(transformation(extent = {{-46, 60}, {-26, 80}}, origin = {6, 0}, rotation = 0), visible = true));
  Modelica.Blocks.Sources.Sine sine2(amplitude = 2, freqHz = 1) annotation(Placement(transformation(extent = {{-100, 60}, {-80, 80}})));
  Parts.Mounting1D mounting1D1(n = {0, 1, 0}) annotation(Placement(transformation(extent = {{-60, 34}, {-40, 54}}, origin = {0, -4}, rotation = 0), visible = true));
  Joints.Revolute r2(useAxisFlange = true, n = {0, 1, 0}) annotation(Placement(transformation(extent = {{-38, -80}, {-18, -60}}, origin = {8, 0}, rotation = 0), visible = true));
  Rotational.Sources.Position position2(useSupport = true, w(fixed = true)) annotation(Placement(transformation(extent = {{-52, -30}, {-32, -10}}, origin = {2, 0}, rotation = 0), visible = true));
  Parts.Mounting1D mounting1D2(n = {0, 1, 0}) annotation(Placement(transformation(extent = {{-66, -56}, {-46, -36}}, origin = {-4, -4}, rotation = 0), visible = true));
  Parts.Fixed fixed annotation(Placement(transformation(extent = {{-86, -80}, {-66, -60}}, origin = {-4, 0}, rotation = 0), visible = true));
equation
  connect(revolute.frame_b, bodyCylinder.frame_a) annotation(Line(points = {{-5, 0}, {5, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {45, 20}));
  connect(torque.frame_b, bodyCylinder.frame_b) annotation(Line(points = {{70, 50}, {80, 50}, {80, 20}, {70, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(sine1.y, torque.torque) annotation(Line(points = {{-8.667, 6}, {4.333, 6}, {4.333, -12}}, color = {1, 37, 163}, visible = true, origin = {49.667, 74}));
  connect(torque1.flange, rotor1D.flange_a) annotation(Line(points = {{-5, 0}, {5, 0}}, visible = true, color = {64, 64, 64}, origin = {45, -30}));
  connect(mounting1D.flange_b, torque1.support) annotation(Line(points = {{20, -50}, {30, -50}, {30, -40}}, visible = true, color = {64, 64, 64}));
  connect(r1.frame_a, world.frame_b) annotation(Line(points = {{20, 0}, {-20, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-40, 20}));
  connect(position1.flange, r1.axis) annotation(Line(points = {{-6.667, 13.333}, {3.333, 13.333}, {3.333, -26.667}}, visible = true, color = {64, 64, 64}, origin = {-13.333, 56.667}));
  connect(position1.support, mounting1D1.flange_b) annotation(Line(points = {{3.333, 13.333}, {3.333, -6.667}, {-6.667, -6.667}}, visible = true, origin = {-33.333, 46.667}));
  connect(mounting1D1.frame_a, world.frame_b) annotation(Line(points = {{3.333, 6.667}, {3.333, -3.333}, {-6.667, -3.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-53.333, 23.333}));
  connect(sine2.y, position1.phi_ref) annotation(Line(points = {{-18.5, 0}, {18.5, 0}}, color = {1, 37, 163}, visible = true, origin = {-60.5, 70}));
  connect(r1.frame_b, revolute.frame_a) annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, 20}));
  connect(torque.frame_resolve, revolute.frame_a) annotation(Line(points = {{64, 60}, {10, 60}, {10, 20}, {20, 20}}, color = {95, 95, 95}, pattern = LinePattern.Dot, visible = true));
  connect(torque.frame_a, revolute.frame_a) annotation(Line(points = {{50, 50}, {10, 50}, {10, 20}, {20, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(position2.flange, r2.axis) annotation(Line(points = {{-6.667, 13.333}, {3.333, 13.333}, {3.333, -26.667}}, visible = true, color = {64, 64, 64}, origin = {-23.333, -33.333}));
  connect(position2.support, mounting1D2.flange_b) annotation(Line(points = {{3.333, 13.333}, {3.333, -6.667}, {-6.667, -6.667}}, visible = true, origin = {-43.333, -43.333}));
  connect(fixed.frame_b, r2.frame_a) annotation(Line(points = {{-20, 0}, {20, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-50, -70}));
  connect(fixed.frame_b, mounting1D2.frame_a) annotation(Line(points = {{-10, -1.564}, {-0, -1.564}, {-0, 8.436}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-60, -68.436}));
  connect(sine2.y, position2.phi_ref) annotation(Line(points = {{-79, 70}, {-70, 70}, {-70, 46}, {-90, 46}, {-90, -20}, {-52, -20}}, color = {1, 37, 163}, visible = true));
  connect(r2.frame_b, rotor1D.frame_a) annotation(Line(points = {{-46.667, -10}, {23.333, -10}, {23.333, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {36.667, -60}));
  connect(r2.frame_b, mounting1D.frame_a) annotation(Line(points = {{-13.333, -3.333}, {6.667, -3.333}, {6.667, 6.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {3.333, -66.667}));
  connect(sine1[1].y, torque1.tau) annotation(Line(points = {{41, 80}, {90, 80}, {90, -10}, {0, -10}, {0, -30}, {18, -30}}, color = {1, 37, 163}, visible = true));
  annotation(experiment(StopTime = 1.1), Documentation(info = "<html>
<p>
This model demonstrates how a moving drive train modelled with 3-dim. multi-body elements
(revolute, bodyCylinder) can alternatively be modeled with component <a href=\"modelica://UWBody.Parts.Rotor1D\">rotor1D</a> to speed up
simulation. The movement of the two systems is identical and also the cut-torques in the
various frames (such as: r1.frame_b.t and r2.frame_b.t).
</p>

<p>
The driving joints (r1, r2) with rotation axis {0,1,0} are modelled to be driven by a motor torque
along the {1,0,0} axis. Basically, this means that an idealized bevel gear is used to drive the
axes of the revolute joints.
</p>
</html>"));
end MovingActuatedDrive;
