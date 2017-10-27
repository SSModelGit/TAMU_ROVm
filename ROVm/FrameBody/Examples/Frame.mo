within ROVm.FrameBody.Examples;

model Frame
  inner RBodyInFluid.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBody basicBody annotation(Placement(visible = true, transformation(origin = {-70, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end Frame;
