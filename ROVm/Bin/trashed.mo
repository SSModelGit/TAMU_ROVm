within ROVm.Bin;

model trashed
  inner UWmBody.UWWorld world annotation(Placement(visible = true, transformation(origin = {-103.187, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  UWmBody.UWParts.Body body(r_CM = {0.01, 0, 0}, m = 5, d = 1200, I_11 = 1, I_22 = 1, I_33 = 1, r_0.start = {0, 0, 0}, r_0.fixed = true, c_d = 0.1) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  annotation(experiment(StopTime = 20, Interval = 0.001, __Wolfram_Algorithm = "rk4"), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end trashed;
