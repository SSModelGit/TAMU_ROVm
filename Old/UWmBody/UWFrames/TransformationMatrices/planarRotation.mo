within UWmBody.UWFrames.TransformationMatrices;

function planarRotation "Return orientation object of a planar rotation"
  import Modelica.Math;
  extends Modelica.Icons.Function;
  input Real e[3](each final unit = "1") "Normalized axis of rotation (must have length=1)";
  input Modelica.SIunits.Angle angle "Rotation angle to rotate frame 1 into frame 2 along axis e";
  output TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
algorithm
  T := [e] * transpose([e]) + (identity(3) - [e] * transpose([e])) * Math.cos(angle) - skew(e) * Math.sin(angle);
  annotation(Inline = true);
end planarRotation;
