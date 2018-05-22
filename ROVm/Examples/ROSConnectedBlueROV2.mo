within ROVm.Examples;

model ROSConnectedBlueROV2
  extends ROVm.Templates.BlueROV2Template;
  UnderwaterRigidBodyLibrary.Controllers.ROSControllerBlock_Joy rOVJoystickController(n = 6, samplePeriod = 0.05) annotation(Placement(visible = true, transformation(origin = {-152.574, 150}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(firstOrder2.y, rOVJoystickController.u[1]) annotation(Line(visible = true, origin = {-109.287, 154.338}, points = {{65.287, -4.338}, {58.287, -4.338}, {58.287, 8.675}, {-58.287, 8.675}, {-58.287, -4.338}, {-55.287, -4.338}}, color = {1, 37, 163}));
  connect(firstOrder1.y, rOVJoystickController.u[2]) annotation(Line(visible = true, origin = {-68.458, 154.338}, points = {{106.116, -4.338}, {99.116, -4.338}, {99.116, 8.675}, {-99.116, 8.675}, {-99.116, -4.338}, {-96.116, -4.338}}, color = {1, 37, 163}));
  connect(firstOrder5.y, rOVJoystickController.u[3]) annotation(Line(visible = true, origin = {-109.287, 67.671}, points = {{65.287, -47.671}, {58.287, -47.671}, {58.287, -34.658}, {-58.287, -34.658}, {-58.287, 82.329}, {-55.287, 82.329}}, color = {1, 37, 163}));
  connect(firstOrder6.y, rOVJoystickController.u[4]) annotation(Line(visible = true, origin = {-59.986, 67.671}, points = {{114.589, -47.671}, {107.589, -47.671}, {107.589, -34.658}, {-107.589, -34.658}, {-107.589, 82.329}, {-104.588, 82.329}}, color = {1, 37, 163}));
  connect(firstOrder4.y, rOVJoystickController.u[5]) annotation(Line(visible = true, origin = {-109.287, 104.337}, points = {{65.287, -29.337}, {58.287, -29.337}, {58.287, -16.325}, {-58.287, -16.325}, {-58.287, 45.663}, {-55.287, 45.663}}, color = {1, 37, 163}));
  connect(firstOrder3.y, rOVJoystickController.u[6]) annotation(Line(visible = true, origin = {-59.986, 107.671}, points = {{114.589, -27.671}, {107.589, -27.671}, {107.589, -14.658}, {-107.589, -14.658}, {-107.589, 42.329}, {-104.588, 42.329}}, color = {1, 37, 163}));
  connect(rOVJoystickController.y[1], firstOrder1.u) annotation(Line(visible = true, origin = {-68.458, 150}, points = {{-73.116, 0}, {83.116, 0}}, color = {1, 37, 163}));
  connect(rOVJoystickController.y[2], firstOrder2.u) annotation(Line(visible = true, origin = {-109.287, 150}, points = {{-32.287, 0}, {42.287, 0}}, color = {1, 37, 163}));
  connect(rOVJoystickController.y[3], firstOrder3.u) annotation(Line(visible = true, origin = {-64.584, 83}, points = {{-76.99, 67}, {-47.692, 67}, {-47.692, -3}, {79.584, -3}, {96.187, -3}}, color = {1, 37, 163}));
  connect(rOVJoystickController.y[4], firstOrder4.u) annotation(Line(visible = true, origin = {-94.644, 112.5}, points = {{-46.93, 37.5}, {14.644, 37.5}, {14.644, -37.5}, {27.644, -37.5}}, color = {1, 37, 163}));
  connect(rOVJoystickController.y[5], firstOrder5.u) annotation(Line(visible = true, origin = {-94.644, 85}, points = {{-46.93, 65}, {14.644, 65}, {14.644, -65}, {27.644, -65}}, color = {1, 37, 163}));
  connect(rOVJoystickController.y[6], firstOrder6.u) annotation(Line(visible = true, origin = {-29.993, 117.5}, points = {{-111.581, 32.5}, {29.993, 32.5}, {29.993, -97.5}, {61.596, -97.5}}, color = {1, 37, 163}));
  annotation(experiment(StopTime = -1, Interval = 0.01, __Wolfram_Algorithm = "cvodes", Tolerance = 1e-3), Diagram(coordinateSystem(extent = {{-220, -220}, {220, 220}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ROSConnectedBlueROV2;
