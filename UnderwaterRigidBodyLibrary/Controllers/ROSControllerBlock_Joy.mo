within UnderwaterRigidBodyLibrary.Controllers;

model ROSControllerBlock_Joy "Provides integration with ROS, allowing control value input by joystick"
  extends Modelica.Blocks.Interfaces.DiscreteMIMOs;
  parameter Integer portNumber = 9090 annotation(Dialog(group = "External Connection Parameters"));
equation
  when sampleTrigger then
    y = UnderwaterRigidBodyLibrary.Controllers.ROS_Controller_Joystick(time, portNumber);
  end when annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ROSControllerBlock_Joy;
