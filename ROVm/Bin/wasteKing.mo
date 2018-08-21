within ROVm.Bin;

model wasteKing
  MultiBody.Parts.Body body(r_CM = {0.01, 0, 0}, m = 5, r_0.start = {0, 0, 0}, r_0.fixed = true, v_0.start = {0, 0, 0}, v_0.fixed = true) annotation(Placement(visible = true, transformation(origin = {0, -22.777}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  MultiBody.Parts.Body body1(r_CM = {0.01, 0, 0}, m = 5) annotation(Placement(visible = true, transformation(origin = {0, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  MultiBody.Parts.FixedTranslation fixedTranslation(r = {1, 0, 0}) annotation(Placement(visible = true, transformation(origin = {-27.609, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  MultiBody.Forces.WorldForce force annotation(Placement(visible = true, transformation(origin = {-51.766, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-130, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(body.frame_a, fixedTranslation.frame_a) annotation(Line(visible = true, origin = {-21.739, -18.518}, points = {{11.739, -4.259}, {-5.87, -4.259}, {-5.87, 8.518}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, body1.frame_a) annotation(Line(visible = true, origin = {-21.739, 16.667}, points = {{-5.87, -6.667}, {-5.87, 3.333}, {11.739, 3.333}}, color = {95, 95, 95}));
  connect(force.frame_b, body1.frame_a) annotation(Line(visible = true, origin = {-25.883, 20}, points = {{-15.883, 0}, {15.883, 0}}, color = {95, 95, 95}));
  force.force = -2 * body1.m * world.g * Modelica.Math.Vectors.normalizeWithAssert(world.n);
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end wasteKing;
