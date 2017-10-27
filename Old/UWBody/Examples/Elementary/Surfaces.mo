within UWBody.Examples.Elementary;

model Surfaces "Demonstrate the visualization of a sine surface, as well as a torus and a wheel constructed from a surface"
  extends Modelica.Icons.Example;
  parameter Real x_min = -1 "Minimum value of x";
  parameter Real x_max = +1 "Maximum value of x";
  parameter Real y_min = -1 "Minimum value of y";
  parameter Real y_max = +1 "Maximum value of y";
  parameter Real z_min = 0 "Minimum value of z";
  parameter Real z_max = 1 "Maximum value of z";
  Real wz = time;
  UWBody.Visualizers.Advanced.Surface surface(redeclare function surfaceCharacteristic = UWBody.Examples.Elementary.Utilities.sineSurface(x_min = x_min, x_max = x_max, y_min = y_min, y_max = y_max, z_min = z_min, z_max = z_max, wz = wz), multiColoredSurface = false, nu = 50, nv = 50) annotation(Placement(transformation(extent = {{-72, -54}, {-52, -34}}, origin = {12, -16}, rotation = 0), visible = true));
  inner World world(axisLength = 1, n = {0, 0, -1}) annotation(Placement(transformation(extent = {{-80, 20}, {-60, 40}}, origin = {20, -20}, rotation = 0), visible = true));
  Visualizers.Torus torus annotation(Placement(transformation(extent = {{32, 20}, {52, 40}}, origin = {28, -20}, rotation = 0), visible = true));
  Joints.Prismatic prismatic(useAxisFlange = true, animation = false, v(fixed = true)) annotation(Placement(transformation(extent = {{-38, 20}, {-18, 40}}, origin = {18, -20}, rotation = 0), visible = true));
  Translational.Sources.Position position annotation(Placement(transformation(extent = {{-66, 70}, {-46, 90}}, origin = {26, -30}, rotation = 0), visible = true));
  Modelica.Blocks.Sources.Sine sine(amplitude = 2, freqHz = 0.5) annotation(Placement(transformation(extent = {{-98, 70}, {-78, 90}}, origin = {18, -30}, rotation = 0), visible = true));
  Visualizers.Ground ground(groundColor = {215, 215, 215}, length = 4) annotation(Placement(transformation(extent = {{-72, -16}, {-52, 4}}, origin = {12, -14}, rotation = 0), visible = true));
  Parts.FixedTranslation fixedTranslation1(r = {0, -1.3, torus.ro + torus.ri}, animation = false) annotation(Placement(transformation(extent = {{-4, 20}, {16, 40}}, origin = {24, -20}, rotation = 0), visible = true));
  Parts.FixedTranslation fixedTranslation2(animation = false, r = {0, -1.6, wheel.rTire}) annotation(Placement(transformation(extent = {{-4, 50}, {16, 70}}, origin = {24, -20}, rotation = 0), visible = true));
  Visualizers.VoluminousWheel wheel annotation(Placement(transformation(extent = {{32, 50}, {52, 70}}, origin = {28, -20}, rotation = 0), visible = true));
  Visualizers.PipeWithScalarField pipeWithScalarField(rOuter = 0.3, length = 1, T_min = 0, T_max = 2, T = sin(Modelica.Constants.pi * pipeWithScalarField.xsi) * cos(Modelica.Constants.pi * time) .+ 1, n_colors = 32) annotation(Placement(transformation(extent = {{14, -30}, {34, -10}}, origin = {16, -10}, rotation = 0), visible = true));
  Parts.FixedTranslation fixedTranslation3(animation = false, r = {0, -2.2, 0}) annotation(Placement(transformation(extent = {{-20, -30}, {0, -10}}, origin = {10, -10}, rotation = 0), visible = true));
equation
  connect(world.frame_b, prismatic.frame_a) annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-30, 10}));
  connect(position.flange, prismatic.axis) annotation(Line(points = {{-38, 70}, {-20, 70}, {-20, 36}}, color = {0, 127, 0}, visible = true, origin = {18, -20}));
  connect(sine.y, position.s_ref) annotation(Line(points = {{-8.5, 0}, {8.5, 0}}, color = {1, 37, 163}, visible = true, origin = {-50.5, 50}));
  connect(prismatic.frame_b, fixedTranslation1.frame_a) annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {10, 10}));
  connect(fixedTranslation1.frame_b, torus.frame_a) annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {50, 10}));
  connect(prismatic.frame_b, fixedTranslation2.frame_a) annotation(Line(points = {{-18, 30}, {-8, 30}, {-8, 60}, {2, 60}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {18, -20}));
  connect(fixedTranslation2.frame_b, wheel.frame_a) annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {50, 40}));
  connect(world.frame_b, fixedTranslation3.frame_a) annotation(Line(points = {{-58, 30}, {-48, 30}, {-48, -10}, {-28, -10}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {18, -20}));
  connect(fixedTranslation3.frame_b, pipeWithScalarField.frame_a) annotation(Line(points = {{-10, 0}, {10, -0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {20, -30}));
  annotation(experiment(StopTime = 5), Documentation(info = "<html>
<p>
This example demonstrates the use of the
<a href=\"modelica://UWBody.Visualizers.Advanced.Surface\">Surface</a>
visualizer that visualizes a moving, parameterized surface.
The \"sine-wave\" surface is a direct application of the surface model.
Furthermore, the \"torus\" surface is an instance of
<a href=\"modelica://UWBody.Visualizers.Torus\">Torus</a>,
the \"wheel\" surface is an instance of
<a href=\"modelica://UWBody.Visualizers.VoluminousWheel\">VoluminousWheel</a>,
and the \"pipeWithScalarField surface is an instance of
<a href=\"modelica://UWBody.Visualizers.PipeWithScalarField\">PipeWithScalarField</a>.
All latter visual shapes are constructed with the surface model.
The following image shows a screen-shot of this example model:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/Surfaces.png\">
</blockquote>

</html>"));
end Surfaces;
