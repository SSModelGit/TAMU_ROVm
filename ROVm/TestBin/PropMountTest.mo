within ROVm.TestBin;

model PropMountTest
  RBodyInFluid.Parts.BasicPropMount basicPropMount annotation(Placement(visible = true, transformation(origin = {7.592, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  /*prop_R = {0.1, 0, 0}*/
  RBodyInFluid.Parts.BasicBody basicBody(density = 1000, c_d = 1, A = 0.5, r_CM = {0.1, 0, 0}, m = 5, r_0.start = {0, 0, 0}, r_0.fixed = true) annotation(Placement(visible = true, transformation(origin = {-35, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RBodyInFluid.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-122.168, -22.432}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.Torque torque annotation(Placement(visible = true, transformation(origin = {17.255, 32.44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine tau(freqHz = 1) annotation(Placement(visible = true, transformation(origin = {-57.978, 68.332}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(basicPropMount.frame_a, basicBody.frame_a) annotation(Line(visible = true, origin = {-25.168, -7.17}, points = {{32.76, -2.657}, {32.76, -5.842}, {-22.844, -5.842}, {-22.844, 7.17}, {-19.832, 7.17}}, color = {95, 95, 95}));
  connect(torque.flange, basicPropMount.flange_b) annotation(Line(visible = true, origin = {24.032, 10.813}, points = {{3.223, 21.627}, {3.223, -10.813}, {-6.447, -10.813}}, color = {64, 64, 64}));
  connect(tau.y, torque.tau) annotation(Line(visible = true, origin = {-10.31, 50.386}, points = {{-36.669, 17.946}, {10.552, 17.946}, {10.552, -17.946}, {15.565, -17.946}}, color = {1, 37, 163}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end PropMountTest;
