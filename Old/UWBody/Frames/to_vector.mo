within UWBody.Frames;

function to_vector "Map rotation object into vector"
  extends Modelica.Icons.Function;
  input Orientation R "Orientation object to rotate frame 1 into frame 2";
  output Real vec[9] "Elements of R in one vector";
algorithm
  vec := {R.T[1, 1], R.T[2, 1], R.T[3, 1], R.T[1, 2], R.T[2, 2], R.T[3, 2], R.T[1, 3], R.T[2, 3], R.T[3, 3]};
  annotation(Inline = true);
end to_vector;
