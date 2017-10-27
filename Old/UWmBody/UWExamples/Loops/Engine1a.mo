within UWmBody.UWExamples.Loops;

model Engine1a "Model of one cylinder engine"
  extends Modelica.Icons.Example;
  UWmBody.UWParts.BodyCylinder Piston(diameter = 0.1, r = {0, -0.1, 0}) annotation(Placement(transformation(origin = {40, 70}, extent = {{-10, 20}, {10, -20}}, rotation = 270), visible = true));
  UWmBody.UWParts.BodyBox Rod(widthDirection = {1, 0, 0}, width = 0.02, height = 0.06, r = {0, -0.2, 0}, color = {0, 0, 200}) annotation(Placement(transformation(origin = {40, 10}, extent = {{10, -10}, {-10, 10}}, rotation = 90), visible = true));
  UWmBody.UWJoints.Revolute B2(n = {1, 0, 0}, cylinderLength = 0.02, cylinderDiameter = 0.05) annotation(Placement(transformation(extent = {{80, 22}, {100, 42}}, origin = {-50, 8}, rotation = 0), visible = true));
  UWmBody.UWJoints.Revolute Bearing(useAxisFlange = true, n = {1, 0, 0}, cylinderLength = 0.02, cylinderDiameter = 0.05) annotation(Placement(transformation(extent = {{-10, -80}, {10, -100}}, origin = {-40, 0}, rotation = 0), visible = true));
  inner UWmBody.UWWorld world annotation(Placement(transformation(extent = {{-50, -100}, {-30, -80}}, origin = {-40, 0}, rotation = 0), visible = true));
  Modelica.Mechanics.Rotational.Components.Inertia Inertia(stateSelect = StateSelect.always, phi(fixed = true, start = 0), w(fixed = true, start = 10), J = 1) annotation(Placement(transformation(extent = {{-28, -120}, {-8, -100}}, origin = {-42, 0}, rotation = 0), visible = true));
  UWmBody.UWParts.BodyBox Crank4(height = 0.05, widthDirection = {1, 0, 0}, width = 0.02, r = {0, -0.1, 0}) annotation(Placement(transformation(origin = {70, -75}, extent = {{10, -10}, {-10, 10}}, rotation = 90), visible = true));
  UWmBody.UWParts.BodyCylinder Crank3(r = {0.1, 0, 0}, diameter = 0.03) annotation(Placement(transformation(extent = {{81.5, -71}, {101.5, -51}}, origin = {-46.5, 1}, rotation = 0), visible = true));
  UWmBody.UWParts.BodyCylinder Crank1(diameter = 0.05, r = {0.1, 0, 0}) annotation(Placement(transformation(extent = {{24, -100}, {44, -80}}, origin = {-44, 0}, rotation = 0), visible = true));
  UWmBody.UWParts.BodyBox Crank2(r = {0, 0.1, 0}, height = 0.05, widthDirection = {1, 0, 0}, width = 0.02) annotation(Placement(transformation(origin = {20, -75}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true));
  UWJoints.RevolutePlanarLoopConstraint B1(n = {1, 0, 0}, cylinderLength = 0.02, cylinderDiameter = 0.05) annotation(Placement(transformation(extent = {{80, -30}, {100, -10}}, origin = {-50, 0}, rotation = 0), visible = true));
  UWmBody.UWParts.FixedTranslation Mid(r = {0.05, 0, 0}) annotation(Placement(transformation(extent = {{70, -53}, {90, -33}}, origin = {-50, -2}, rotation = 0), visible = true));
  UWmBody.UWJoints.Prismatic Cylinder(boxWidth = 0.02, n = {0, -1, 0}, s(start = 0.15)) annotation(Placement(transformation(origin = {40, 95}, extent = {{-10, -10}, {10, 10}}, rotation = 270), visible = true));
  UWmBody.UWParts.FixedTranslation cylPosition(animation = false, r = {0.15, 0.45, 0}) annotation(Placement(transformation(extent = {{-0.5, 100}, {19.5, 120}}, origin = {-39.5, 0}, rotation = 0), visible = true));
equation
  connect(world.frame_b, Bearing.frame_a) annotation(Line(points = {{-30, -90}, {-10, -90}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-40, 0}));
  connect(Crank2.frame_a, Crank1.frame_b) annotation(Line(points = {{10, 3.333}, {10, -1.667}, {-10, -1.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, -88.333}));
  connect(Crank2.frame_b, Crank3.frame_a) annotation(Line(points = {{-5, -3.333}, {-5, 1.667}, {10, 1.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {25, -61.667}));
  connect(Bearing.frame_b, Crank1.frame_a) annotation(Line(points = {{-5, 0}, {5, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-25, -90}));
  connect(cylPosition.frame_b, Cylinder.frame_a) annotation(Line(points = {{-36.667, 1.667}, {23.333, 1.667}, {23.333, -3.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {16.667, 108.333}));
  connect(world.frame_b, cylPosition.frame_a) annotation(Line(points = {{-30, -90}, {-20, -90}, {-20, 110}, {0, 110}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-40, 0}));
  connect(Crank3.frame_b, Crank4.frame_a) annotation(Line(points = {{-10, 1.667}, {5, 1.667}, {5, -3.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {65, -61.667}));
  connect(B1.frame_a, Mid.frame_b) annotation(Line(points = {{80, -20}, {70, -20}, {70, -35}, {100, -35}, {100, -45}, {90, -45}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-50, 0}));
  connect(B1.frame_b, Rod.frame_b) annotation(Line(points = {{100, -20}, {110, -20}, {110, -5}, {90, -5}, {90, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-50, 0}));
  connect(Rod.frame_a, B2.frame_b) annotation(Line(points = {{90, 20}, {90, 25}, {110, 25}, {110, 40}, {100, 40}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-50, 0}));
  connect(B2.frame_a, Piston.frame_b) annotation(Line(points = {{80, 40}, {70, 40}, {70, 55}, {90, 55}, {90, 60}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-50, 0}));
  connect(Inertia.flange_b, Bearing.axis) annotation(Line(points = {{-10, -110}, {0, -110}, {0, -100}}, visible = true, origin = {-40, 0}, color = {64, 64, 64}));
  connect(Mid.frame_a, Crank2.frame_b) annotation(Line(points = {{70, -45}, {60, -45}, {60, -60}, {70, -60}, {70, -65}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-50, 0}));
  connect(Cylinder.frame_b, Piston.frame_a) annotation(Line(points = {{90, 85}, {90, 77}, {90, 80}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-50, 0}));
  annotation(Diagram(coordinateSystem(extent = {{-130, -130}, {130, 130}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})), experiment(StopTime = 5), Documentation(info = "<html>
<p>
This is a model of the mechanical part of one cylinder of an engine.
The combustion is not modelled. The \"inertia\" component at the lower
left part is the output inertia of the engine driving the gearbox.
The angular velocity of the output inertia has a start value of 10 rad/s
in order to demonstrate the movement of the engine.
</p>
<p>
The engine is modeled solely by revolute and prismatic joints.
Since this results in a <b>planar</b> loop there is the well known
difficulty that the cut-forces perpendicular to the loop cannot be
uniquely computed, as well as the cut-torques within the plane.
This ambiguity is resolved by using the option <b>planarCutJoint</b>
in the <b>Advanced</b> menu of one revolute joint in every planar loop
(here: joint B1). This option sets the cut-force in direction of the
axis of rotation, as well as the cut-torques perpendicular to the axis
of rotation at this joint to zero and makes the problem mathematically
well-formed.
</p>
<p>
An animation of this example is shown in the figure below.
</p>

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Loops/Engine.png\" ALT=\"model Examples.Loops.Engine\">
</html>"));
end Engine1a;
