within UWBody.Frames.Quaternions;

function resolve2 "Transform vector from frame 1 to frame 2"
  extends Modelica.Icons.Function;
  input Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
  input Real v1[3] "Vector in frame 1";
  output Real v2[3] "Vector in frame 2";
algorithm
  v2 := 2 * ((Q[4] * Q[4] - 0.5) * v1 + Q[1:3] * v1 * Q[1:3] - Q[4] * cross(Q[1:3], v1));
  annotation(Inline = true);
end resolve2;
