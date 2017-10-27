within UWBody.Examples.Constraints;

model RevoluteConstraint "Body attached by one spring and revolute joint or constrained to environment"
  extends Modelica.Icons.Example;
  parameter Boolean animation = true "= true, if animation shall be enabled";
  UWBody.Joints.Revolute joint(stateSelect = StateSelect.never, n = {0, 1, 0}, phi(fixed = true), w(fixed = true)) annotation(Placement(transformation(extent = {{60, -30}, {40, -10}}, origin = {0, 10}, rotation = 0), visible = true));
  Joints.Constraints.Revolute constraint(n = joint.n) annotation(Placement(transformation(extent = {{60, 10}, {40, 30}}, origin = {0, 10}, rotation = 0), visible = true));
  UWBody.Sensors.RelativeSensor sensorConstraintRelative(resolveInFrame = UWBody.Types.ResolveInFrameAB.frame_a, get_r_rel = true, get_a_rel = false, get_angles = true) annotation(Placement(transformation(extent = {{60, 60}, {40, 40}}, origin = {0, 10}, rotation = 0), visible = true));
  UWBody.Parts.BodyShape bodyOfJoint(m = 1, I_11 = 1, I_22 = 1, I_33 = 1, r = {0.4, 0, 0}, r_CM = {0.2, 0, 0}, width = 0.05, r_0(start = {0.2, -0.5, 0.1}, each fixed = false), v_0(each fixed = false), angles_fixed = false, w_0_fixed = false, angles_start = {0.17453292519943, 0.95993108859688, 1.1868238913561}, final color = {0, 0, 255}) annotation(Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-10, -10}), visible = true));
  UWBody.Parts.BodyShape bodyOfConstraint(I_11 = 1, I_22 = 1, I_33 = 1, width = 0.05, r_0(start = {0.2, -0.5, 0.1}, each fixed = false), v_0(each fixed = false), angles_fixed = false, w_0_fixed = false, final color = {0, 128, 0}, r = bodyOfJoint.r, r_CM = bodyOfJoint.r_CM, m = bodyOfJoint.m, angles_start = {0.17453292519943, 0.95993108859688, 1.1868238913561}) annotation(Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {-10, 30}), visible = true));
  UWBody.Forces.Spring springOfJoint(c = 20, s_unstretched = 0, width = 0.1, coilWidth = 0.005, numberOfWindings = 5) annotation(Placement(transformation(origin = {-50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0), visible = true));
  UWBody.Forces.Spring springOfConstraint(width = 0.1, coilWidth = 0.005, c = springOfJoint.c, s_unstretched = springOfJoint.s_unstretched, numberOfWindings = springOfJoint.numberOfWindings) annotation(Placement(transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), visible = true));
  inner UWBody.World world annotation(Placement(transformation(extent = {{-100, -100}, {-80, -80}}, origin = {0, 20}, rotation = 0), visible = true));
  UWBody.Parts.FixedRotation fixedRotation(r = {0.2, -0.3, 0.2}, rotationType = UWBody.Types.RotationTypes.PlanarRotationSequence, angles = {10, 55, 68}) annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {80, -40}), visible = true));
  UWBody.Parts.FixedTranslation fixedTranslation(animation = false, r = {0.8, 0, 0.3}) annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-70, -40}), visible = true));
  Joints.FreeMotionScalarInit freeMotionScalarInit(use_angle = true, use_angle_d = true, angle_2(start = 0, fixed = true), angle_d_2(start = 0, fixed = true)) annotation(Placement(transformation(extent = {{40, 60}, {20, 80}}, origin = {0, 10}, rotation = 0), visible = true));
equation
  connect(fixedTranslation.frame_a, world.frame_b) annotation(Line(points = {{3.333, 13.333}, {3.333, -6.667}, {-6.667, -6.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-73.333, -63.333}));
  connect(bodyOfConstraint.frame_b, springOfConstraint.frame_b) annotation(Line(points = {{-20, 20}, {-40, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, 10}));
  connect(world.frame_b, fixedRotation.frame_a) annotation(Line(points = {{-106.667, -6.667}, {53.333, -6.667}, {53.333, 13.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {26.667, -63.333}));
  connect(fixedRotation.frame_b, constraint.frame_a) annotation(Line(points = {{80, -40}, {80, 20}, {60, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, 10}));
  connect(constraint.frame_a, sensorConstraintRelative.frame_a) annotation(Line(points = {{60, 20}, {80, 20}, {80, 50}, {60, 50}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, 10}));
  connect(bodyOfJoint.frame_b, springOfJoint.frame_b) annotation(Line(points = {{-20, -20}, {-40, -20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, 10}));
  connect(joint.frame_b, bodyOfJoint.frame_a) annotation(Line(points = {{40, -20}, {0, -20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, 10}));
  connect(sensorConstraintRelative.frame_b, constraint.frame_b) annotation(Line(points = {{40, 50}, {30, 50}, {30, 20}, {40, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, 10}));
  connect(fixedTranslation.frame_b, springOfJoint.frame_a) annotation(Line(points = {{-70, -40}, {-70, -20}, {-60, -20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, 10}));
  connect(fixedTranslation.frame_b, springOfConstraint.frame_a) annotation(Line(points = {{-70, -40}, {-70, 20}, {-60, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, 10}));
  connect(bodyOfConstraint.frame_a, constraint.frame_b) annotation(Line(points = {{0, 20}, {40, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, 10}));
  connect(joint.frame_a, fixedRotation.frame_b) annotation(Line(points = {{60, -20}, {80, -20}, {80, -40}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, 10}));
  connect(freeMotionScalarInit.frame_a, fixedRotation.frame_b) annotation(Line(points = {{40, 70}, {80, 70}, {80, -40}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, 10}));
  connect(bodyOfConstraint.frame_a, freeMotionScalarInit.frame_b) annotation(Line(points = {{0, 20}, {10, 20}, {10, 70}, {20, 70}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, 10}));
  annotation(experiment(StopTime = 10), Documentation(info = "<html>
<p>This example demonstrates the functionality of <b>constraint</b> representing <b>revolute joint</b>. Each of two bodies is at one of its end connected by spring to the world. The other end is also connected to the world either by revolute joint or by appropriate constraint. Therefore, the body can only perform rotation about the revolute axis depending on working forces.</p>
<p><b>Simulation results</b> </p>
<p>After simulating the model, see the animation of the UWBody system and compare movement of body connected by joint (blue colored) with movement of that one connected by constraint (of green color). Additionally, the outputs from <code>sensorConstraintRelative</code> depict both position and angle deviations in the constraining element.</p>
</html>"));
end RevoluteConstraint;
