within UWmBody.UWFrames.Quaternions;

function planarRotation "Return quaternion orientation object of a planar rotation"
  import Modelica.Math;
  extends Modelica.Icons.Function;
  input Real e[3](each final unit = "1") "Normalized axis of rotation (must have length=1)";
  input Modelica.SIunits.Angle angle "Rotation angle to rotate frame 1 into frame 2 along axis e";
  output Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2 along axis e";
algorithm
  Q := vector([e * Math.sin(angle / 2); Math.cos(angle / 2)]);
  annotation(Inline = true);
end planarRotation;
