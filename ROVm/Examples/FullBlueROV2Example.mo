within ROVm.Examples;

model FullBlueROV2Example
  extends ROVm.Templates.BlueROV2Template(redeclare replaceable Modelica.Blocks.Continuous.SecondOrder firstOrder2(w = 1, D = 1, k = forwardProfile.k2), redeclare replaceable Modelica.Blocks.Continuous.SecondOrder firstOrder1(k = forwardProfile.k1, w = 1, D = 1), redeclare replaceable Modelica.Blocks.Continuous.SecondOrder firstOrder4(k = forwardProfile.k4, w = 1, D = 1), redeclare replaceable Modelica.Blocks.Continuous.SecondOrder firstOrder3(k = forwardProfile.k3, w = 1, D = 1), redeclare replaceable Modelica.Blocks.Continuous.SecondOrder firstOrder5(k = forwardProfile.k5, w = 1, D = 1), redeclare replaceable Modelica.Blocks.Continuous.SecondOrder firstOrder6(k = forwardProfile.k6, w = 1, D = 1));
  // battery density was 0.81
  replaceable Modelica.Blocks.Sources.Pulse power(amplitude = 14.8, startTime = 10, period = 10) constrainedby Modelica.Blocks.Interfaces.SO annotation(Placement(visible = true, transformation(origin = {-110, 150}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter ConstantMotionProfiles.ForwardMotion forwardProfile "Record containing gain parameters for a forward motion profile" annotation(Dialog(group = "Motion Profiles"));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity annotation(Placement(visible = true, transformation(origin = {170, 45}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity annotation(Placement(visible = true, transformation(origin = {170, -46.751}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.SIunits.Acceleration lin_a[3];
  Modelica.SIunits.AngularAcceleration ang_a[3];
equation
  lin_a = der(absoluteVelocity.v);
  ang_a = der(absoluteAngularVelocity.w);
  connect(power.y, firstOrder1.u) annotation(Line(visible = true, origin = {-42.171, 150}, points = {{-56.829, 0}, {56.829, 0}}, color = {1, 37, 163}));
  connect(power.y, firstOrder2.u) annotation(Line(visible = true, origin = {-83, 150}, points = {{-16, 0}, {16, 0}}, color = {1, 37, 163}));
  connect(power.y, firstOrder3.u) annotation(Line(visible = true, origin = {-24.247, 116.656}, points = {{-74.753, 33.344}, {23.918, 33.344}, {23.918, -33.343}, {26.918, -33.343}}, color = {1, 37, 163}));
  connect(power.y, firstOrder4.u) annotation(Line(visible = true, origin = {-76.5, 112.5}, points = {{-22.5, 37.5}, {6.5, 37.5}, {6.5, -37.5}, {9.5, -37.5}}, color = {1, 37, 163}));
  connect(power.y, firstOrder5.u) annotation(Line(visible = true, origin = {-76.5, 85}, points = {{-22.5, 65}, {6.5, 65}, {6.5, -65}, {9.5, -65}}, color = {1, 37, 163}));
  connect(power.y, firstOrder6.u) annotation(Line(visible = true, origin = {-24.247, 86.656}, points = {{-74.753, 63.344}, {23.918, 63.344}, {23.918, -63.343}, {26.918, -63.343}}, color = {1, 37, 163}));
  connect(rov_origin.frame_b, absoluteVelocity.frame_a) annotation(Line(visible = true, origin = {153.5, 32.5}, points = {{-13.5, -12.5}, {3.5, -12.5}, {3.5, 12.5}, {6.5, 12.5}}, color = {95, 95, 95}));
  connect(rov_COM.frame_b, absoluteAngularVelocity.frame_a) annotation(Line(visible = true, origin = {153.5, -35.875}, points = {{-13.5, 10.875}, {3.5, 10.875}, {3.5, -10.875}, {6.5, -10.875}}, color = {95, 95, 95}));
  annotation(experiment(StopTime = 20.0, __Wolfram_Algorithm = "dassl", Tolerance = 1e-3), Diagram(coordinateSystem(extent = {{-220, -220}, {220, 220}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end FullBlueROV2Example;
