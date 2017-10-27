within UWBody.Examples.Rotational3DEffects;

model GearConstraint "Demonstrate usage of GearConstraint model"
  extends Modelica.Icons.Example;
  Joints.GearConstraint gearConstraint(ratio = 10, phi_b(fixed = true), w_b(fixed = true), checkTotalPower = true) annotation(Placement(transformation(extent = {{34, 40}, {54, 60}}, origin = {6, 0}, rotation = 0), visible = true));
  inner World world(driveTrainMechanics3D = true, g = 0) annotation(Placement(transformation(extent = {{-62, 10}, {-42, 30}}, origin = {2, 0}, rotation = 0), visible = true));
  Parts.BodyCylinder cyl1(diameter = 0.1, color = {0, 128, 0}, r = {0.4, 0, 0}) annotation(Placement(transformation(extent = {{2, 40}, {22, 60}}, origin = {8, 0}, rotation = 0), visible = true));
  Parts.BodyCylinder cyl2(r = {0.4, 0, 0}, diameter = 0.2) annotation(Placement(transformation(extent = {{70, 40}, {90, 60}}, origin = {0, 0}, rotation = 0), visible = true));
  Forces.Torque torque1 annotation(Placement(transformation(extent = {{-26, 40}, {-6, 60}}, origin = {6, 0}, rotation = 0), visible = true));
  Modelica.Blocks.Sources.Sine sine[3](amplitude = {2, 0, 0}, freqHz = {1, 1, 1}) annotation(Placement(transformation(extent = {{-100, 60}, {-80, 80}}, origin = {0, 0}, rotation = 0), visible = true));
  Parts.Fixed fixed annotation(Placement(transformation(extent = {{-48, -90}, {-28, -70}}, origin = {-2, 10}, rotation = 0), visible = true));
  Rotational.Components.Inertia inertia1(J = cyl1.I[1, 1], a(fixed = false), phi(fixed = true, start = 0), w(fixed = true, start = 0)) annotation(Placement(transformation(extent = {{-20, -40}, {0, -20}}, origin = {0, 10}, rotation = 0), visible = true));
  Rotational.Components.IdealGear idealGear(ratio = 10, useSupport = true) annotation(Placement(transformation(extent = {{12, -40}, {32, -20}}, origin = {-2, 10}, rotation = 0), visible = true));
  Rotational.Components.Inertia inertia2(J = cyl2.I[1, 1]) annotation(Placement(transformation(extent = {{44, -40}, {64, -20}}, origin = {-4, 10}, rotation = 0), visible = true));
  Rotational.Sources.Torque torque2(useSupport = true) annotation(Placement(transformation(extent = {{-48, -40}, {-28, -20}}, origin = {-2, 10}, rotation = 0), visible = true));
  Parts.Mounting1D mounting1D annotation(Placement(transformation(extent = {{-20, -70}, {0, -50}}, origin = {-10, 10}, rotation = 0), visible = true));
equation
  connect(world.frame_b, gearConstraint.bearing) annotation(Line(points = {{-60, -6.667}, {30, -6.667}, {30, 13.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {20, 26.667}));
  connect(cyl1.frame_b, gearConstraint.frame_a) annotation(Line(points = {{-5, -0}, {5, -0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {35, 50}));
  connect(gearConstraint.frame_b, cyl2.frame_a) annotation(Line(points = {{-5, -0}, {5, -0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {65, 50}));
  connect(torque1.frame_b, cyl1.frame_a) annotation(Line(points = {{-5, -0}, {5, -0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {5, 50}));
  connect(torque1.frame_a, world.frame_b) annotation(Line(points = {{-20, 50}, {-30, 50}, {-30, 20}, {-40, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(sine.y, torque1.torque) annotation(Line(points = {{-42, 2.667}, {21, 2.667}, {21, -5.333}}, color = {1, 37, 163}, visible = true, origin = {-37, 67.333}));
  connect(inertia1.flange_b, idealGear.flange_a) annotation(Line(points = {{-5, 0}, {5, 0}}, visible = true, color = {64, 64, 64}, origin = {5, -20}));
  connect(idealGear.flange_b, inertia2.flange_a) annotation(Line(points = {{-5, 0}, {5, 0}}, visible = true, color = {64, 64, 64}, origin = {35, -20}));
  connect(torque2.flange, inertia1.flange_a) annotation(Line(points = {{-5, 0}, {5, 0}}, visible = true, color = {64, 64, 64}, origin = {-25, -20}));
  connect(sine[1].y, torque2.tau) annotation(Line(points = {{-79, 70}, {-70, 70}, {-70, -20}, {-52, -20}}, color = {1, 37, 163}, visible = true));
  connect(mounting1D.flange_b, idealGear.support) annotation(Line(points = {{-20, -6.667}, {10, -6.667}, {10, 13.333}}, visible = true, color = {64, 64, 64}, origin = {10, -43.333}));
  connect(mounting1D.flange_b, torque2.support) annotation(Line(points = {{-10, -60}, {0, -60}, {0, -50}, {-40, -50}, {-40, -40}}, visible = true, color = {64, 64, 64}, origin = {0, 10}));
  connect(fixed.frame_b, mounting1D.frame_a) annotation(Line(points = {{-10, -3.333}, {0, -3.333}, {0, 6.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-20, -66.667}));
  annotation(experiment(StopTime = 5), Documentation(info = "<html>
<p>
This model demonstrates the usage of the
<a href=\"UWBody.Joints.GearConstraint\">GearConstraint</a>
model to model a gearbox with multi-body elements. The formulation is compared with
a one-dimensional model that is mounted with an
<a href=\"modelica://UWBody.Parts.Mounting1D\">Mounting1D</a> element.
</p>
</html>"));
end GearConstraint;
