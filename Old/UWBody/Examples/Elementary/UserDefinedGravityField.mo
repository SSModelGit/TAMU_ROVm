within UWBody.Examples.Elementary;

model UserDefinedGravityField "Demonstrate the modeling of a user-defined gravity field"
  extends Modelica.Icons.Example;
  parameter Modelica.SIunits.Conversions.NonSIunits.Angle_deg geodeticLatitude = 0 "Geodetic latitude";
  parameter Modelica.SIunits.Position height = 20 "Height of pendulum attachment point over WGS84 earth ellipsoid";
  Modelica.SIunits.Acceleration gravity[3] = body.g_0 "Gravity acceleration at center of mass of body";
  inner UWBody.World world(gravityType = UWBody.Types.GravityTypes.NoGravity, redeclare function gravityAcceleration = UWBody.Examples.Elementary.Utilities.theoreticalNormalGravityWGS84(phi = geodeticLatitude), axisLength = 10, nominalLength = 10) annotation(Placement(transformation(extent = {{-80, -20}, {-60, 0}}, origin = {10, -30}, rotation = 0), visible = true));
  Joints.Revolute rev(n = {0, 0, 1}, useAxisFlange = true, phi(fixed = true), w(fixed = true)) annotation(Placement(transformation(extent = {{-14, 20}, {6, 40}}, origin = {4, -30}, rotation = 0), visible = true));
  Rotational.Components.Damper damper(d = 0.1) annotation(Placement(transformation(extent = {{-14, 60}, {6, 80}}, origin = {4, -30}, rotation = 0), visible = true));
  Parts.Body body(r_CM = {10, 0, 0}, m = 1000.0, sphereDiameter = 1) annotation(Placement(transformation(extent = {{26, 20}, {46, 40}}, origin = {4, -30}, rotation = 0), visible = true));
  Parts.FixedTranslation fixedTranslation(r = {0, height, 0}, width = 0.3) annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-30, -20}), visible = true));
equation
  connect(damper.flange_b, rev.axis) annotation(Line(points = {{6, 66}, {16, 66}, {16, 46}, {-4, 46}, {-4, 36}}, visible = true, origin = {4, -26}, color = {64, 64, 64}));
  connect(rev.support, damper.flange_a) annotation(Line(points = {{-10, 40}, {-10, 50}, {-24, 50}, {-24, 70}, {-14, 70}}, visible = true, origin = {4, -30}, color = {64, 64, 64}));
  connect(body.frame_a, rev.frame_b) annotation(Line(points = {{26, 30}, {6, 30}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {4, -30}));
  connect(world.frame_b, fixedTranslation.frame_a) annotation(Line(points = {{-13.333, -3.333}, {6.667, -3.333}, {6.667, 6.667}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-36.667, -36.667}));
  connect(fixedTranslation.frame_b, rev.frame_a) annotation(Line(points = {{-6.667, -6.667}, {-6.667, 3.333}, {13.333, 3.333}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-23.333, -3.333}));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Angle of revolute", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = rev.phi, legend = "RevoluteAngle")})})})), experiment(StopTime = 10, Tolerance = 1e-008), Documentation(info = "<html>
<p>
This example demonstrates a user defined gravity field.
Function \"world.gravityAcceleration\" is redeclared to function
<a href=\"modelica://UWBody.Examples.Elementary.Utilities.theoreticalNormalGravityWGS84\">theoreticalNormalGravityWGS84</a>
that computes the theoretical gravity of the
<a href=\"http://earth-info.nga.mil/GandG/publications/tr8350.2/wgs84fin.pdf\">WGS84 ellipsoid earth model</a> at and close to
the earth ellipsoid surface. In the gravity field, a large, single pendulum is present. Via parameter \"geodeticLatitude\", the geodetic latitude on the earth can be defined, where the pendulum is present. The world frame is located at the WGS84 earth ellipsoid at this latitude. The result variable
\"gravity\" is the gravity vector at the center of mass of the pendulum mass.
Since the height of this mass is changing, the value of the gravity is also changing
(the difference is in the order of 0.00001).
</p>

<p>
The result of the simulation is slightly different at the equator (geodeticLatitude=0)
and at the poles (geodeticLatitude=90). For example, after 10 s of simulation time
the rotation angle of the pendulum, rev.phi, has the following values:
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><td><b><i>latitude [deg]</i></b></td>
    <td><b><i>rev.phi [rad]</i></b></td></tr>
<tr><td> = 0</td>
    <td>= -2.39 rad</td></tr>

<tr><td>= 90</td>
    <td>= -2.42 rad</td></tr>
</table>
</html>"));
end UserDefinedGravityField;
