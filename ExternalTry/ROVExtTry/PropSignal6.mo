within ExternalTry.ROVExtTry;

function PropSignal6
  extends Modelica.Icons.Function;
  input Real c "Constant signal input";
  output Real res[6] "output[6] signal";

  external "C" PropSignal6(c, res) annotation(Include = "#include \"PropSignal6.c\"", IncludeDirectory = "modelica://ExternalTry/Resources/");
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end PropSignal6;
