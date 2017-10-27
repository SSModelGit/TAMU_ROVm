within UWmBody.UWFrames.TransformationMatrices;

function resolve1 "Transform vector from frame 2 to frame 1"
  extends Modelica.Icons.Function;
  input TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
  input Real v2[3] "Vector in frame 2";
  output Real v1[3] "Vector in frame 1";
algorithm
  v1 := transpose(T) * v2;
  annotation(Inline = true);
end resolve1;
