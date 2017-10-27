within UWBody.Frames.TransformationMatrices;

function orientationConstraint "Return residues of orientation constraints (shall be zero)"
  extends Modelica.Icons.Function;
  input TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
  output Real residue[6] "Residues of constraints between elements of orientation object (shall be zero)";
algorithm
  residue := {T[:, 1] * T[:, 1] - 1, T[:, 2] * T[:, 2] - 1, T[:, 3] * T[:, 3] - 1, T[:, 1] * T[:, 2], T[:, 1] * T[:, 3], T[:, 2] * T[:, 3]};
  annotation(Inline = true);
end orientationConstraint;
