within ExternalTry.ROSConnectTry;

function ROSClientCall_Array
  extends Modelica.Icons.Function;
  input Real t "time";
  input Integer port "port number";
  input Real query1 "something random";
  input Real query2 "something random - second strand";
  output Real res[2] "output signal";

  external "C" ROSClientCall_Array(t, port, query1, query2, res) annotation(Include = "#include \"ROSClientCall_Array.c\"", IncludeDirectory = "modelica://ExternalTry/Resources/");
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ROSClientCall_Array;
