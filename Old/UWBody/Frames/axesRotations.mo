within UWBody.Frames;

function axesRotations "Return fixed rotation object to rotate in sequence around fixed angles along 3 axes"
  import TM = UWBody.Frames.TransformationMatrices;
  extends Modelica.Icons.Function;
  input Integer sequence[3](min = {1, 1, 1}, max = {3, 3, 3}) = {1, 2, 3} "Sequence of rotations from frame 1 to frame 2 along axis sequence[i]";
  input Modelica.SIunits.Angle angles[3] "Rotation angles around the axes defined in 'sequence'";
  input Modelica.SIunits.AngularVelocity der_angles[3] "= der(angles)";
  output Orientation R "Orientation object to rotate frame 1 into frame 2";
algorithm
  /*
      R := absoluteRotation(absoluteRotation(axisRotation(sequence[1], angles[1],
        der_angles[1]), axisRotation(sequence[2], angles[2], der_angles[2])),
        axisRotation(sequence[3], angles[3], der_angles[3]));
    */
  R := Orientation(T = TM.axisRotation(sequence[3], angles[3]) * TM.axisRotation(sequence[2], angles[2]) * TM.axisRotation(sequence[1], angles[1]), w = Frames.axis(sequence[3]) * der_angles[3] + TM.resolve2(TM.axisRotation(sequence[3], angles[3]), Frames.axis(sequence[2]) * der_angles[2]) + TM.resolve2(TM.axisRotation(sequence[3], angles[3]) * TM.axisRotation(sequence[2], angles[2]), Frames.axis(sequence[1]) * der_angles[1]));
  annotation(Inline = true, Documentation(info = "<html>
<h4>Syntax</h4>
<blockquote><pre>
R = Frames.<b>axesRotation</b>(asequence, angles, der_angles);
</pre></blockquote>

<h4>Description</h4>
<p>
The function call <code>Frames.<b>axesRotation</b>(sequence, angles, der_angles)</code> returns
orientation object R that describes the orientation to rotate
along unit axis <b>axis</b>
from frame 1 into frame 2 with angle <b>angle</b> and derivative of angle <b>der_angle</b>.
For example, Frames.axisRotation(2, phi, der_phi) returns the same orientation object as with the call
Frames.planarRotation({0,1,0}, phi, der_phi)
</p>
</html>"));
end axesRotations;
