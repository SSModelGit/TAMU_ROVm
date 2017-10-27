within UWBody.Frames.TransformationMatrices;

function from_nxy "Return orientation object from n_x and n_y vectors"
  extends Modelica.Icons.Function;
  input Real n_x[3](each final unit = "1") "Vector in direction of x-axis of frame 2, resolved in frame 1";
  input Real n_y[3](each final unit = "1") "Vector in direction of y-axis of frame 2, resolved in frame 1";
  output TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
protected
  Real abs_n_x = sqrt(n_x * n_x);
  Real e_x[3](each final unit = "1") = if abs_n_x < 1.e-10 then {1, 0, 0} else n_x / abs_n_x;
  Real n_z_aux[3](each final unit = "1") = cross(e_x, n_y);
  Real n_y_aux[3](each final unit = "1") = if n_z_aux * n_z_aux > 1.0e-6 then n_y else if abs(e_x[1]) > 1.0e-6 then {0, 1, 0} else {1, 0, 0};
  Real e_z_aux[3](each final unit = "1") = cross(e_x, n_y_aux);
  Real e_z[3](each final unit = "1") = e_z_aux / sqrt(e_z_aux * e_z_aux);
algorithm
  T := {e_x, cross(e_z, e_x), e_z};
  annotation(Documentation(info = "<html>
<p>
It is assumed that the two input vectors n_x and n_y are
resolved in frame 1 and are directed along the x and y axis
of frame 2 (i.e., n_x and n_y are orthogonal to each other)
The function returns the orientation object T to rotate from
frame 1 to frame 2.
</p>
<p>
The function is robust in the sense that it returns always
an orientation object T, even if n_y is not orthogonal to n_x.
This is performed in the following way:
</p>
<p>
If n_x and n_y are not orthogonal to each other, first a unit
vector e_y is determined that is orthogonal to n_x and is lying
in the plane spanned by n_x and n_y. If n_x and n_y are parallel
or nearly parallel to each other, a vector e_y is selected
arbitrarily such that e_x and e_y are orthogonal to each other.
</p>
</html>"));
end from_nxy;
