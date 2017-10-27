within UWBody.Frames.Quaternions;

function inverseRotation "Return inverse quaternions orientation object"
  extends Modelica.Icons.Function;
  input Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
  output Quaternions.Orientation Q_inv "Quaternions orientation object to rotate frame 2 into frame 1";
algorithm
  Q_inv := {-Q[1], -Q[2], -Q[3], Q[4]};
  annotation(Inline = true);
end inverseRotation;
