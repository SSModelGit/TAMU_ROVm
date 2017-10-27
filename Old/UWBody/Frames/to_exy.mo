within UWBody.Frames;

function to_exy "Map rotation object into e_x and e_y vectors of frame 2, resolved in frame 1"
  extends Modelica.Icons.Function;
  input Orientation R "Orientation object to rotate frame 1 into frame 2";
  output Real exy[3, 2] "= [e_x, e_y] where e_x and e_y are axes unit vectors of frame 2, resolved in frame 1";
algorithm
  exy := [R.T[1, :], R.T[2, :]];
  annotation(Inline = true);
end to_exy;
