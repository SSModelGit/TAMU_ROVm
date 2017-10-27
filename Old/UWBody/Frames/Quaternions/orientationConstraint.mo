within UWBody.Frames.Quaternions;

function orientationConstraint "Return residues of orientation constraints (shall be zero)"
  extends Modelica.Icons.Function;
  input Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
  output Real residue[1] "Residue constraint (shall be zero)";
algorithm
  residue := {Q * Q - 1};
  annotation(Inline = true);
end orientationConstraint;
