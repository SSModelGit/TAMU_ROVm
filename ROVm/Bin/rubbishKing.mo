within ROVm.Bin;

model rubbishKing
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-135, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner UnderwaterRigidBodyLibrary.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-135, -32.785}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  UnderwaterRigidBodyLibrary.Parts.BasicBodyShape basicBody1(c_d = 0, A = 0.5, r_CM = {0, 0, 1}, m = 5, density = 2000, r = {0, 0, 1}, r_0.fixed = true, r_0.start = {0, 0, 0}) annotation(Placement(visible = true, transformation(origin = {-30, -25}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape(r = {0, 0, 1}, r_CM = {0, 0, 1}, m = 5, r_0.fixed = true, r_0.start = {2, 0, 0}) annotation(Placement(visible = true, transformation(origin = {-30, -52.111}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end rubbishKing;
