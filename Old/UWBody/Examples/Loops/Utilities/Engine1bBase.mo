within UWBody.Examples.Loops.Utilities;

partial model Engine1bBase "Model of one cylinder engine with gas force"
  UWBody.Parts.BodyCylinder Piston(diameter = 0.1, r = {0, -0.1, 0}) annotation(Placement(transformation(origin = {120, 50}, extent = {{-10, 20}, {10, -20}}, rotation = 270), visible = true));
  UWBody.Parts.BodyBox Rod2(widthDirection = {1, 0, 0}, width = 0.02, height = 0.06, color = {0, 0, 200}, r = {0, 0.2, 0}) annotation(Placement(transformation(origin = {120, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true));
  UWBody.Joints.Revolute Bearing(useAxisFlange = true, n = {1, 0, 0}, cylinderLength = 0.02, cylinderDiameter = 0.05) annotation(Placement(transformation(extent = {{-50, -80}, {-30, -100}})));
  inner UWBody.World world annotation(Placement(transformation(extent = {{-90, -100}, {-70, -80}})));
  Modelica.Mechanics.Rotational.Components.Inertia Inertia(stateSelect = StateSelect.always, J = 0.1, w(fixed = true), phi(fixed = true, start = 0.001, displayUnit = "rad")) annotation(Placement(transformation(extent = {{-68, -120}, {-48, -100}}, origin = {-2, 0}, rotation = 0), visible = true));
  UWBody.Parts.BodyBox Crank4(height = 0.05, widthDirection = {1, 0, 0}, width = 0.02, r = {0, -0.1, 0}) annotation(Placement(transformation(origin = {80, -75}, extent = {{10, -10}, {-10, 10}}, rotation = 90), visible = true));
  UWBody.Parts.BodyCylinder Crank3(r = {0.1, 0, 0}, diameter = 0.03) annotation(Placement(transformation(extent = {{41.5, -71}, {61.5, -51}}, origin = {3.5, 1}, rotation = 0), visible = true));
  UWBody.Parts.BodyCylinder Crank1(diameter = 0.05, r = {0.1, 0, 0}) annotation(Placement(transformation(extent = {{-16, -100}, {4, -80}}, origin = {-4, 0}, rotation = 0), visible = true));
  UWBody.Parts.BodyBox Crank2(height = 0.05, widthDirection = {1, 0, 0}, width = 0.02, r = {0, 0.1, 0}) annotation(Placement(transformation(origin = {30, -75}, extent = {{-10, -10}, {10, 10}}, rotation = 90), visible = true));
  UWBody.Parts.FixedTranslation Mid(r = {0.05, 0, 0}) annotation(Placement(transformation(extent = {{30, -53}, {50, -33}}, origin = {0, -2}, rotation = 0), visible = true));
  UWBody.Parts.FixedTranslation cylPosition(animation = false, r = {0.15, 0.55, 0}) annotation(Placement(transformation(extent = {{-40.5, 100}, {-20.5, 120}}, origin = {0.5, 0}, rotation = 0), visible = true));
  Modelica.Utilities.GasForce2 gasForce(d = 0.1, L = 0.35) annotation(Placement(transformation(origin = {120, 95}, extent = {{10, -10}, {-10, 10}}, rotation = 90), visible = true));
equation
  connect(world.frame_b, Bearing.frame_a) annotation(Line(points = {{-70, -90}, {-50, -90}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(Crank2.frame_a, Crank1.frame_b) annotation(Line(points = {{10, 3.333}, {10, -1.667}, {-20, -1.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {20, -88.333}));
  connect(Crank2.frame_b, Crank3.frame_a) annotation(Line(points = {{-5, -3.333}, {-5, 1.667}, {10, 1.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {35, -61.667}));
  connect(Bearing.frame_b, Crank1.frame_a) annotation(Line(points = {{-5, 0}, {5, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-25, -90}));
  connect(world.frame_b, cylPosition.frame_a) annotation(Line(points = {{-70, -90}, {-60, -90}, {-60, 110}, {-40, 110}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(Crank3.frame_b, Crank4.frame_a) annotation(Line(points = {{65, -60}, {80, -60}, {80, -65}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(Inertia.flange_b, Bearing.axis) annotation(Line(points = {{-50, -110}, {-40, -110}, {-40, -100}}, visible = true, color = {64, 64, 64}));
  connect(Mid.frame_a, Crank2.frame_b) annotation(Line(points = {{30, -45}, {20, -45}, {20, -60}, {30, -60}, {30, -65}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})), Documentation(info = "<html>
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
end Engine1bBase;
