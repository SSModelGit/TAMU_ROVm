within ROVm.TestBin;

model TopPlateTest
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-125, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner UnderwaterRigidBodyLibrary.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-130, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FrameBody.TopFrame.TopPiece topPiece(c_d_TP = 100, c_d_Fairing = 100, d_plate.displayUnit = "g/cm3", A_TP.displayUnit = "m2", r_0.start = {0, 0, 0}, r_0.fixed = true) annotation(Placement(visible = true, transformation(origin = {-20, -2.416}, extent = {{-10, -10}, {10, 10}}, rotation = -360)));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end TopPlateTest;
