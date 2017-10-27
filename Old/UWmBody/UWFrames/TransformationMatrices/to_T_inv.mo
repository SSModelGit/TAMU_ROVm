within UWmBody.UWFrames.TransformationMatrices;

function to_T_inv "Return inverse transformation matrix T_inv from orientation object R"
  extends Modelica.Icons.Function;
  input TransformationMatrices.Orientation R "Orientation object to rotate frame 1 into frame 2";
  output Real T_inv[3, 3] "Inverse transformation matrix to transform vector from frame 2 into frame 1 (v1=T_inv*v2)";
algorithm
  T_inv := transpose(R);
  annotation(Inline = true);
end to_T_inv;
