within ROVm.Controllers;

function ROVClientCall_Joystick
  extends Modelica.Icons.Function;
  input Real t "time";
  input Integer port "port number";
  output Real res[6] "output signal";

  external "C" ROVClientCall_Joystick(t, port, res) annotation(Include = "#include \"ROVClientCall_Joystick.c\"", IncludeDirectory = "modelica://ROVm/Resources/");
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ROVClientCall_Joystick;
