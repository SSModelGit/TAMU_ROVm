within UWBody.Frames.Quaternions;

function absoluteRotation "Return absolute quaternions orientation object from another absolute and a relative quaternions orientation object"
  extends Modelica.Icons.Function;
  input Quaternions.Orientation Q1 "Quaternions orientation object to rotate frame 0 into frame 1";
  input Quaternions.Orientation Q_rel "Quaternions orientation object to rotate frame 1 into frame 2";
  output Quaternions.Orientation Q2 "Quaternions orientation object to rotate frame 0 into frame 2";
algorithm
  Q2 := [Q_rel[4], Q_rel[3], -Q_rel[2], Q_rel[1]; -Q_rel[3], Q_rel[4], Q_rel[1], Q_rel[2]; Q_rel[2], -Q_rel[1], Q_rel[4], Q_rel[3]; -Q_rel[1], -Q_rel[2], -Q_rel[3], Q_rel[4]] * Q1;
  annotation(Inline = true);
end absoluteRotation;
