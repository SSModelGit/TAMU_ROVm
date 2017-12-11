within ExternalTry;

function ExtCall
  input Modelica.SIunits.AngularVelocity w_start "Angular velocity at start time";
  input Modelica.SIunits.AngularVelocity w_end "Angular velocity at end time";
  input Real A "amplitude of signal";
  input Real M "time period for signal";
  input Real t "time";
  output Real u "output signal";

  external "C"  annotation(Include = "#include \"ExtCall.c\"", IncludeDirectory = "modelica://ExternalTry/Resources/");
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ExtCall;
