within ROVm.TestBin;

model BottomPlateTest
  FrameBody.BottomFrame.BottomPlate blueROV2(animationFT = false) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner UnderwaterRigidBodyLibrary.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-132.522, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-132.522, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  annotation(experiment(__Wolfram_Algorithm = "cvodes", Tolerance = 1e-3), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end BottomPlateTest;
