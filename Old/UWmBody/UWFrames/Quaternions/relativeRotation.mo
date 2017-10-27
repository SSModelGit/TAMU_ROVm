within UWmBody.UWFrames.Quaternions;

function relativeRotation "Return relative quaternions orientation object"
  extends Modelica.Icons.Function;
  input Quaternions.Orientation Q1 "Quaternions orientation object to rotate frame 0 into frame 1";
  input Quaternions.Orientation Q2 "Quaternions orientation object to rotate frame 0 into frame 2";
  output Quaternions.Orientation Q_rel "Quaternions orientation object to rotate frame 1 into frame 2";
algorithm
  Q_rel := [Q1[4], Q1[3], -Q1[2], -Q1[1]; -Q1[3], Q1[4], Q1[1], -Q1[2]; Q1[2], -Q1[1], Q1[4], -Q1[3]; Q1[1], Q1[2], Q1[3], Q1[4]] * Q2;
  annotation(Inline = true);
end relativeRotation;
