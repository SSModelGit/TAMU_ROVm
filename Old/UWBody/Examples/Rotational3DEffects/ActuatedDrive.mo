within UWBody.Examples.Rotational3DEffects;

model ActuatedDrive "Demonstrates usage of models Rotor1D and Mounting1D"
  extends Modelica.Icons.Example;
  Parts.BodyShape bodyCylinder(r = {0.5, 0, 0}, m = 0, I_11 = 2, I_22 = 0, I_33 = 0, shapeType = "cylinder", width = 0.1, animateSphere = false, r_shape = {0.1, 0, 0}, r_CM = {0, 0, 0}) annotation(Placement(transformation(extent = {{8, 0}, {28, 20}}, origin = {22, 10}, rotation = 0), visible = true));
  Joints.Revolute revolute(n = {1, 0, 0}, a(fixed = false), phi(fixed = true), w(fixed = true)) annotation(Placement(transformation(extent = {{-26, 0}, {-6, 20}}, origin = {26, 10}, rotation = 0), visible = true));
  inner World world(g = 0, driveTrainMechanics3D = true) annotation(Placement(transformation(extent = {{-60, 0}, {-40, 20}}, origin = {20, 10}, rotation = 0), visible = true));
  Forces.Torque torque annotation(Placement(transformation(extent = {{8, 30}, {28, 50}}, origin = {22, 10}, rotation = 0), visible = true));
  Modelica.Blocks.Sources.Sine sine[3](amplitude = {1, 0, 0}, freqHz = {1, 1, 1}) annotation(Placement(transformation(extent = {{-80, 60}, {-60, 80}}, origin = {20, 10}, rotation = 0), visible = true));
  inner Parts.Fixed fixed annotation(Placement(transformation(extent = {{-62, -90}, {-42, -70}}, origin = {12, 10}, rotation = 0), visible = true));
  Parts.Rotor1D rotor1D(J = 2, phi(fixed = true), w(fixed = true)) annotation(Placement(transformation(extent = {{0, -40}, {20, -20}}, origin = {20, 10}, rotation = 0), visible = true));
  Rotational.Sources.Torque torque1(useSupport = true) annotation(Placement(transformation(extent = {{-32, -40}, {-12, -20}}, origin = {22, 10}, rotation = 0), visible = true));
  Parts.Mounting1D mounting1D annotation(Placement(transformation(extent = {{-46, -60}, {-26, -40}}, origin = {16, 10}, rotation = 0), visible = true));
equation
  connect(world.frame_b, revolute.frame_a) annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-10, 20}));
  connect(revolute.frame_b, bodyCylinder.frame_a) annotation(Line(points = {{-5, 0}, {5, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {25, 20}));
  connect(torque.frame_b, bodyCylinder.frame_b) annotation(Line(points = {{30, 40}, {40, 40}, {40, 10}, {30, 10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {20, 10}));
  connect(torque.frame_resolve, world.frame_b) annotation(Line(points = {{24, 50}, {24, 60}, {-30, 60}, {-30, 10}, {-40, 10}}, color = {95, 95, 95}, pattern = LinePattern.Dot, visible = true, origin = {20, 10}));
  connect(torque.frame_a, world.frame_b) annotation(Line(points = {{10, 40}, {-30, 40}, {-30, 10}, {-40, 10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {20, 10}));
  connect(sine.y, torque.torque) annotation(Line(points = {{-48.667, 6}, {24.333, 6}, {24.333, -12}}, color = {1, 37, 163}, visible = true, origin = {9.667, 74}));
  connect(fixed.frame_b, rotor1D.frame_a) annotation(Line(points = {{-40, -13.333}, {20, -13.333}, {20, 26.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, -56.667}));
  connect(torque1.flange, rotor1D.flange_a) annotation(Line(points = {{-5, 0}, {5, 0}}, visible = true, origin = {15, -20}, color = {64, 64, 64}));
  connect(mounting1D.flange_b, torque1.support) annotation(Line(points = {{-30, -50}, {-20, -50}, {-20, -40}}, visible = true, origin = {20, 10}, color = {64, 64, 64}));
  connect(mounting1D.frame_a, fixed.frame_b) annotation(Line(points = {{3.333, 13.333}, {3.333, -6.667}, {-6.667, -6.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-23.333, -63.333}));
  connect(sine[1].y, torque1.tau) annotation(Line(points = {{-59, 70}, {-50, 70}, {-50, 40}, {-70, 40}, {-70, -30}, {-32, -30}}, color = {1, 37, 163}, visible = true, origin = {20, 10}));
  annotation(experiment(StopTime = 1.1), Documentation(info = "<html>
<p>
This example demonstrates how to utilize the
<a href=\"modelica://UWBody.Parts.Rotor1D\">Rotor1D</a>
and <a href=\"modelica://UWBody.Parts.Mounting1D\">Mounting1D</a> models and
compares the implementation with a standard multi-body implementation.
Note, the solution with Rotor1D is much more efficient.
</p>
</html>"));
end ActuatedDrive;
