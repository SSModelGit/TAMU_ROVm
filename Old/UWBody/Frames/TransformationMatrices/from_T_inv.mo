within UWBody.Frames.TransformationMatrices;

function from_T_inv "Return orientation object R from inverse transformation matrix T_inv"
  extends Modelica.Icons.Function;
  input Real T_inv[3, 3] "Inverse transformation matrix to transform vector from frame 2 to frame 1 (v1=T_inv*v2)";
  output TransformationMatrices.Orientation R "Orientation object to rotate frame 1 into frame 2";
algorithm
  R := transpose(T_inv);
  annotation(Inline = true);
end from_T_inv;
