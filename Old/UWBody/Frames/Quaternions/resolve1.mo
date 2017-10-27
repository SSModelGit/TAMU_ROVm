within UWBody.Frames.Quaternions;

function resolve1 "Transform vector from frame 2 to frame 1"
  extends Modelica.Icons.Function;
  input Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
  input Real v2[3] "Vector in frame 2";
  output Real v1[3] "Vector in frame 1";
algorithm
  v1 := 2 * ((Q[4] * Q[4] - 0.5) * v2 + Q[1:3] * v2 * Q[1:3] + Q[4] * cross(Q[1:3], v2));
  annotation(Inline = true);
end resolve1;
