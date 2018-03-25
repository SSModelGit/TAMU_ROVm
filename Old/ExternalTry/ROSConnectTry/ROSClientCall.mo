within ExternalTry.ROSConnectTry;

function ROSClientCall
  input Real t "time";
  input Integer port "port number";
  input Real que "something random";
  output Real u "output signal";

  external "C"  annotation(Include = "#include \"ROSClientCall_3.c\"", IncludeDirectory = "modelica://ExternalTry/Resources/");
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ROSClientCall;
