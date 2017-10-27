within UWmBody.UWFrames.Quaternions;

function multipleResolve2 "Transform several vectors from frame 1 to frame 2"
  extends Modelica.Icons.Function;
  input Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
  input Real v1[3, :] "Vectors in frame 1";
  output Real v2[3, size(v1, 2)] "Vectors in frame 2";
algorithm
  v2 := ((2 * Q[4] * Q[4] - 1) * identity(3) + 2 * ([Q[1:3]] * transpose([Q[1:3]]) - Q[4] * skew(Q[1:3]))) * v1;
  annotation(Inline = true);
end multipleResolve2;
