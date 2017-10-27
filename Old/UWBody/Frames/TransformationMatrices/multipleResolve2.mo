within UWBody.Frames.TransformationMatrices;

function multipleResolve2 "Transform several vectors from frame 1 to frame 2"
  extends Modelica.Icons.Function;
  input TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
  input Real v1[3, :] "Vectors in frame 1";
  output Real v2[3, size(v1, 2)] "Vectors in frame 2";
algorithm
  v2 := T * v1;
  annotation(Inline = true);
end multipleResolve2;
