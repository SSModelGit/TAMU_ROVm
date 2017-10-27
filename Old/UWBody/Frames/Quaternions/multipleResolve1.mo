within UWBody.Frames.Quaternions;

function multipleResolve1 "Transform several vectors from frame 2 to frame 1"
  extends Modelica.Icons.Function;
  input Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
  input Real v2[3, :] "Vectors in frame 2";
  output Real v1[3, size(v2, 2)] "Vectors in frame 1";
algorithm
  v1 := ((2 * Q[4] * Q[4] - 1) * identity(3) + 2 * ([Q[1:3]] * transpose([Q[1:3]]) + Q[4] * skew(Q[1:3]))) * v2;
  annotation(Inline = true);
end multipleResolve1;
