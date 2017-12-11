within ExternalTry.ROVExtTry;

function PropSignal
  extends Modelica.Icons.Function;
  input Real c "Constant signal input";
  output Real u "output signal";

  external "C"  annotation(Include = "#include \"PropSignal.c\"", IncludeDirectory = "modelica://ExternalTry/Resources/");
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end PropSignal;
