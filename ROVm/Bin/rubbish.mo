within ROVm.Bin;

model rubbish
  UBody.WBody body(density = 200, r_CM = {0.01, 0, 0}, m = 5, r_0_fixed = true, v_0_fixed = true, r_0_start = {0, 0, 0}, v_0_start = {0, 0, 0}, c_d = 1, a_wb = 0.5) annotation(Placement(visible = true, transformation(origin = {0, -22.777}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner UBody.WaterF waterF annotation(Placement(visible = true, transformation(origin = {-120, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  UBody.WBody body1(density = 1200, r_CM = {0.01, 0, 0}, m = 5, c_d = 1, a_wb = 0.5) annotation(Placement(visible = true, transformation(origin = {0, 5}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {1, 0, 0}) annotation(Placement(visible = true, transformation(origin = {-25, -7.247}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
equation
  connect(body.frame_a, fixedTranslation.frame_a) annotation(Line(visible = true, origin = {-20, -20.934}, points = {{10, -1.843}, {-5, -1.843}, {-5, 3.687}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, body1.frame_a) annotation(Line(visible = true, origin = {-20, 4.251}, points = {{-5, -1.498}, {-5, 0.749}, {10, 0.749}}, color = {95, 95, 95}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end rubbish;
