within ExternalTry.ROSConnectTry;

model ROSBasedBlock_Array
  extends Modelica.Blocks.Interfaces.DiscreteMIMOs;
  parameter Integer portNumber = 9090 annotation(Dialog(group = "External Connection Parameters"));
equation
  when sampleTrigger then
    y = ExternalTry.ROSConnectTry.ROSClientCall_Array(time, portNumber, u[1], u[2]);
  end when annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ROSBasedBlock_Array;
