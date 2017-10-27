within UWBody.Examples.Elementary;

model PointGravityWithPointMasses "Two point masses in a point gravity field (rotation of bodies is neglected)"
  extends Modelica.Icons.Example;
  inner UWBody.World world(mue = 1, gravitySphereDiameter = 0.1, gravityType = UWBody.Types.GravityTypes.PointGravity) annotation(Placement(transformation(extent = {{-80, -20}, {-60, 0}}, origin = {0, 10}, rotation = 0), visible = true));
  UWBody.Parts.PointMass body1(m = 1, sphereDiameter = 0.1, r_0(start = {0, 0.6, 0}, each fixed = true), v_0(start = {1, 0, 0}, each fixed = true)) annotation(Placement(transformation(extent = {{-20, 20}, {0, 40}}, origin = {-10, -50}, rotation = 0), visible = true));
  UWBody.Parts.PointMass body2(m = 1, sphereDiameter = 0.1, r_0(start = {0.6, 0.6, 0}, each fixed = true), v_0(start = {0.6, 0, 0}, each fixed = true)) annotation(Placement(transformation(extent = {{20, 20}, {40, 40}}, origin = {-10, -50}, rotation = 0), visible = true));
  UWBody.Parts.PointMass body3(m = 1, sphereDiameter = 0.1, r_0(start = {0, 0.8, 0}, each fixed = true), v_0(start = {0.6, 0, 0}, each fixed = true)) annotation(Placement(transformation(extent = {{-20, 60}, {0, 80}}, origin = {-10, -50}, rotation = 0), visible = true));
  UWBody.Parts.PointMass body4(m = 1, sphereDiameter = 0.1, r_0(start = {0.3, 0.8, 0}, each fixed = true), v_0(start = {0.6, 0, 0}, each fixed = true)) annotation(Placement(transformation(extent = {{20, 60}, {40, 80}}, origin = {-10, -50}, rotation = 0), visible = true));
  Forces.Spring spring(showMass = false, c = 10, fixedRotationAtFrame_b = true, fixedRotationAtFrame_a = true) annotation(Placement(transformation(extent = {{0, 60}, {20, 80}}, origin = {-10, -50}, rotation = 0), visible = true));
equation
  connect(spring.frame_a, body3.frame_a) annotation(Line(points = {{0, 70}, {-10, 70}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-10, -50}));
  connect(spring.frame_b, body4.frame_a) annotation(Line(points = {{20, 70}, {30, 70}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-10, -50}));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Body positions", preferred = true, subPlots = {SubPlot(curves = {Curve(x = body1.r_0[1], y = body1.r_0[2], legend = "Position of body1 in the x-y plane"), Curve(x = body2.r_0[1], y = body2.r_0[2], legend = "Position of body2 in the x-y plane")}), SubPlot(curves = {Curve(x = body3.r_0[1], y = body3.r_0[2], legend = "Position of body3 in the x-y plane"), Curve(x = body4.r_0[1], y = body4.r_0[2], legend = "Position of body4 in the x-y plane")})})})), experiment(StopTime = 2), Documentation(info = "<html>
<p>
This model demonstrates the usage of model Parts.PointMass in a
point gravity field. The PointMass model has the feature that
that rotation is not taken into account and can therefore also not be
calculated. This example demonstrates two cases where this does not matter:
If a PointMass is not connected (body1, body2), the orientation object in
these point masses is set to a unit rotation.
If a PointMass is connected by a line force element, such as
the used Forces.LineForceWithMass component, then the orientation object
is set to a unit rotation within the line force element.
These are the two cases where the rotation is automatically set to
a default value, when the physical system does not provide the equations.
</p>

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/PointGravityWithPointMasses.png\">
</html>"));
end PointGravityWithPointMasses;
