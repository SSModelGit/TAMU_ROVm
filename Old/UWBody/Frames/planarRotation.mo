within UWBody.Frames;

function planarRotation "Return orientation object of a planar rotation"
  import Modelica.Math;
  extends Modelica.Icons.Function;
  input Real e[3](each final unit = "1") "Normalized axis of rotation (must have length=1)";
  input Modelica.SIunits.Angle angle "Rotation angle to rotate frame 1 into frame 2 along axis e";
  input Modelica.SIunits.AngularVelocity der_angle "= der(angle)";
  output Orientation R "Orientation object to rotate frame 1 into frame 2";
algorithm
  R := Orientation(T = [e] * transpose([e]) + (identity(3) - [e] * transpose([e])) * Modelica.Math.cos(angle) - skew(e) * Modelica.Math.sin(angle), w = e * der_angle);
  annotation(Inline = true, Documentation(info = "<html>
<h4>Syntax</h4>
<blockquote><pre>
R = Frames.<b>planarRotation</b>(e, angle, der_angle);
</pre></blockquote>

<h4>Description</h4>
<p>
The function call <code>Frames.<b>planarRotation</b>(e, angle, der_angle)</code> returns
orientation object R that describes the orientation to rotate in the plane along unit
axis <b>e</b> from frame 1 into frame 2 with angle <b>angle</b> and derivative of angle <b>der_angle</b>.
Note, \"e\" must be a unit vector. However, this is not checked in this function and the function will
return a wrong result, if length(e) is not one.
</p>
</html>"));
end planarRotation;
