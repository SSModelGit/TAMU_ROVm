within UWBody.Frames.TransformationMatrices;

function from_T "Return orientation object R from transformation matrix T"
  extends Modelica.Icons.Function;
  input Real T[3, 3] "Transformation matrix to transform vector from frame 1 to frame 2 (v2=T*v1)";
  output TransformationMatrices.Orientation R "Orientation object to rotate frame 1 into frame 2";
algorithm
  R := T;
  annotation(Inline = true);
end from_T;
