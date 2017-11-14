within ROVm.TestBin;

model SidePlateTest
  inner RBodyInFluid.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-130, -17.601}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-130, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  FrameBody.SideFrame.SidePlate sidePlate(lumenUPos = {0, 0.5, 0}, propVPos = {0.5, 0.3, 0}, TP_Pos = {0.25, 0, 0}, BP_Pos = {0.4, 0, 0}) annotation(Placement(visible = true, transformation(origin = {-45, 3.451}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end SidePlateTest;
