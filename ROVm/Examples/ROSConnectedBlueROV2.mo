within ROVm.Examples;

model ROSConnectedBlueROV2
  extends ROVm.Templates.BlueROV2Template;
  ROS_Bridge.Blocks.ROS_Sampler rOS_Sampler(nin = 6, nout = 6, samplePeriod = 0.05) annotation(Placement(visible = true, transformation(origin = {-90, 190}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(rOS_Sampler.y[1], firstOrder1.u) annotation(Line(visible = true, origin = {-10.256, 170}, points = {{-68.743, 20}, {21.915, 20}, {21.915, -20}, {24.915, -20}}, color = {1, 37, 163}));
  connect(rOS_Sampler.y[2], firstOrder2.u) annotation(Line(visible = true, origin = {-71.5, 170}, points = {{-7.5, 20}, {1.5, 20}, {1.5, -20}, {4.5, -20}}, color = {1, 37, 163}));
  connect(rOS_Sampler.y[3], firstOrder3.u) annotation(Line(visible = true, origin = {-19.247, 136.657}, points = {{-59.753, 53.343}, {18.918, 53.343}, {18.918, -53.343}, {21.918, -53.343}}, color = {1, 37, 163}));
  connect(rOS_Sampler.y[4], firstOrder4.u) annotation(Line(visible = true, origin = {-71.5, 132.5}, points = {{-7.5, 57.5}, {1.5, 57.5}, {1.5, -57.5}, {4.5, -57.5}}, color = {1, 37, 163}));
  connect(rOS_Sampler.y[5], firstOrder5.u) annotation(Line(visible = true, origin = {-71.5, 105}, points = {{-7.5, 85}, {1.5, 85}, {1.5, -85}, {4.5, -85}}, color = {1, 37, 163}));
  connect(rOS_Sampler.y[6], firstOrder6.u) annotation(Line(visible = true, origin = {-19.247, 106.657}, points = {{-59.753, 83.343}, {18.918, 83.343}, {18.918, -83.343}, {21.918, -83.343}}, color = {1, 37, 163}));
  connect(rov_position.r[1], rOS_Sampler.u[1]) annotation(Line(visible = true, origin = {39.5, 81}, points = {{141.5, -61}, {144.5, -61}, {144.5, -48}, {-144.5, -48}, {-144.5, 109}, {-141.5, 109}}, color = {1, 37, 163}));
  connect(rov_position.r[2], rOS_Sampler.u[2]) annotation(Line(visible = true, origin = {39.5, 81}, points = {{141.5, -61}, {144.5, -61}, {144.5, -48}, {-144.5, -48}, {-144.5, 109}, {-141.5, 109}}, color = {1, 37, 163}));
  connect(rov_position.r[3], rOS_Sampler.u[3]) annotation(Line(visible = true, origin = {39.5, 81}, points = {{141.5, -61}, {144.5, -61}, {144.5, -48}, {-144.5, -48}, {-144.5, 109}, {-141.5, 109}}, color = {1, 37, 163}));
  connect(rov_angles.angles[1], rOS_Sampler.u[4]) annotation(Line(visible = true, origin = {39.5, 51}, points = {{141.5, -76}, {144.5, -76}, {144.5, -63}, {-144.5, -63}, {-144.5, 139}, {-141.5, 139}}, color = {1, 37, 163}));
  connect(rov_angles.angles[2], rOS_Sampler.u[5]) annotation(Line(visible = true, origin = {39.5, 51}, points = {{141.5, -76}, {144.5, -76}, {144.5, -63}, {-144.5, -63}, {-144.5, 139}, {-141.5, 139}}, color = {1, 37, 163}));
  connect(rov_angles.angles[3], rOS_Sampler.u[6]) annotation(Line(visible = true, origin = {39.5, 51}, points = {{141.5, -76}, {144.5, -76}, {144.5, -63}, {-144.5, -63}, {-144.5, 139}, {-141.5, 139}}, color = {1, 37, 163}));
  annotation(experiment(StopTime = -1, Interval = 0.01, __Wolfram_Algorithm = "cvodes", Tolerance = 1e-3), Diagram(coordinateSystem(extent = {{-220, -220}, {220, 220}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ROSConnectedBlueROV2;
