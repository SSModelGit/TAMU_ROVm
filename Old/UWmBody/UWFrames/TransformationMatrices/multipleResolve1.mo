within UWmBody.UWFrames.TransformationMatrices;

function multipleResolve1 "Transform several vectors from frame 2 to frame 1"
  extends Modelica.Icons.Function;
  input TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
  input Real v2[3, :] "Vectors in frame 2";
  output Real v1[3, size(v2, 2)] "Vectors in frame 1";
algorithm
  v1 := transpose(T) * v2;
  annotation(Inline = true);
end multipleResolve1;
