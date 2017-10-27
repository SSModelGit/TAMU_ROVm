within UWBody.Frames.TransformationMatrices;

function resolve2 "Transform vector from frame 1 to frame 2"
  extends Modelica.Icons.Function;
  input TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
  input Real v1[3] "Vector in frame 1";
  output Real v2[3] "Vector in frame 2";
algorithm
  v2 := T * v1;
  annotation(Inline = true);
end resolve2;
