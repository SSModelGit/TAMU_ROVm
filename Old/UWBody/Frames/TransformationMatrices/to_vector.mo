within UWBody.Frames.TransformationMatrices;

function to_vector "Map rotation object into vector"
  extends Modelica.Icons.Function;
  input TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
  output Real vec[9] "Elements of T in one vector";
algorithm
  vec := {T[1, 1], T[2, 1], T[3, 1], T[1, 2], T[2, 2], T[3, 2], T[1, 3], T[2, 3], T[3, 3]};
  annotation(Inline = true);
end to_vector;
