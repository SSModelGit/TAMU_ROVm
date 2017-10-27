within UWmBody.UWVisualizers.Advanced.SurfaceCharacteristics;

function torus "Function defining the surface characteristic of a torus"
  extends UWmBody.UWInterfaces.partialSurfaceCharacteristic(final multiColoredSurface = false);
  input Modelica.SIunits.Radius ri = 1 "Inner radius of torus" annotation(Dialog);
  input Modelica.SIunits.Radius ro = 0.2 "Outer radius of torus (=width/2)" annotation(Dialog);
  input Modelica.SIunits.Angle opening = 0 "Opening angle of torus" annotation(Dialog);
  input Modelica.SIunits.Angle startAngle = -Modelica.Constants.pi "Start angle of torus slice" annotation(Dialog);
  input Modelica.SIunits.Angle stopAngle = Modelica.Constants.pi "End angle of torus slice" annotation(Dialog);
protected
  Modelica.SIunits.Angle alpha;
  Modelica.SIunits.Angle beta;
  Modelica.SIunits.Angle phi_start;
  Modelica.SIunits.Angle phi_stop;
algorithm
  phi_start := (-Modelica.Constants.pi) + opening;
  phi_stop := Modelica.Constants.pi - opening;
  for i in 1:nu loop
    alpha := startAngle + (stopAngle - startAngle) * (i - 1) / (nu - 1);
    for j in 1:nv loop
      beta := phi_start + (phi_stop - phi_start) * (j - 1) / (nv - 1);
      X[i, j] := (ri + ro * cos(beta)) * sin(alpha);
      Y[i, j] := ro * sin(beta);
      Z[i, j] := (ri + ro * cos(beta)) * cos(alpha);
    end for;
  end for;
  annotation(Documentation(info = "<html>
<p>
Function <b>torus</b> computes the X,Y,Z arrays to visualize a torus
with model <a href=\"modelica://Modelica.Mechanics.MultiBody.Visualizers.Torus\">Torus</a>.
The left image below shows a torus with ri=0.5 m and ro = 0.2 m.
The right images below shows the torus with the additional parameter
settings:
</p>
<pre>
  opening    =   45 degree
  startAngle = -135 degree
  stopAngle  =  135 degree
</pre>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Visualizers/Torus.png\">
</blockquote>
</html>"));
end torus;
