within UWBody.Frames;

function axisRotation "Return rotation object to rotate around an angle along one frame axis"
  extends Modelica.Icons.Function;
  input Integer axis(min = 1, max = 3) "Rotate around 'axis' of frame 1";
  input Modelica.SIunits.Angle angle "Rotation angle to rotate frame 1 into frame 2 along 'axis' of frame 1";
  input Modelica.SIunits.AngularVelocity der_angle "= der(angle)";
  output Orientation R "Orientation object to rotate frame 1 into frame 2";
algorithm
  R := Orientation(T = if axis == 1 then [1, 0, 0; 0, cos(angle), sin(angle); 0, -sin(angle), cos(angle)] else if axis == 2 then [cos(angle), 0, -sin(angle); 0, 1, 0; sin(angle), 0, cos(angle)] else [cos(angle), sin(angle), 0; -sin(angle), cos(angle), 0; 0, 0, 1], w = if axis == 1 then {der_angle, 0, 0} else if axis == 2 then {0, der_angle, 0} else {0, 0, der_angle});
  annotation(Inline = true, Documentation(info = "<html>
<h4>Syntax</h4>
<blockquote><pre>
R = Frames.<b>axisRotation</b>(axis, angle, der_angle);
</pre></blockquote>

<h4>Description</h4>
<p>
The function call <code>Frames.<b>axisRotation</b>(axis, angle, der_angle)</code> returns
orientation object R that describes the orientation to rotate along unit axis <b>axis</b>
from frame 1 into frame 2 with angle <b>angle</b> and derivative of angle <b>der_angle</b>.
For example, Frames.axisRotation(2, phi, der_phi) returns the same orientation object as with the call
Frames.planarRotation({0,1,0}, phi, der_phi)
</p>
</html>"));
end axisRotation;
