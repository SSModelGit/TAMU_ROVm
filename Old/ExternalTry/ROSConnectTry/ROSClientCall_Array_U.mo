within ExternalTry.ROSConnectTry;

function ROSClientCall_Array_U
  extends Modelica.Icons.Function;
  input Real t "time";
  input Integer port "port number";
  input Real query[2] "something random";
  output Real res[2] "output signal";

  external "C" ROSClientCall_Array_U(t, port, query, res) annotation(Include = "#include \"ROSClientCall_Array_U.c\"", IncludeDirectory = "modelica://ExternalTry/Resources/");
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ROSClientCall_Array_U;
