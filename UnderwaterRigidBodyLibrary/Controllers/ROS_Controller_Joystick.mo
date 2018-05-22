within UnderwaterRigidBodyLibrary.Controllers;

function ROS_Controller_Joystick
  extends Modelica.Icons.Function;
  input Real t "time";
  input Integer port "port number";
  output Real res[6] "output signal";

  external "C" ROS_Controller_Joystick(t, port, res) annotation(Include = "#include \"ROS_Controller_Joystick.c\"", IncludeDirectory = "modelica://UnderwaterRigidBodyLibrary/Resources/");
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ROS_Controller_Joystick;
