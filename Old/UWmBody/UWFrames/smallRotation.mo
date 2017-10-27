within UWmBody.UWFrames;

function smallRotation "Return rotation angles valid for a small rotation and optionally residues that should be zero"
  extends Modelica.Icons.Function;
  input Orientation R "Orientation object to rotate frame 1 into frame 2";
  input Boolean withResidues = false "= false/true, if 'angles'/'angles and residues' are returned in phi";
  output Modelica.SIunits.Angle phi[if withResidues then 6 else 3] "The rotation angles around x-, y-, and z-axis of frame 1 to rotate frame 1 into frame 2 for a small rotation + optionally 3 residues that should be zero";
algorithm
  /* Planar rotation:
           Trel = [e]*transpose([e]) + (identity(3) - [e]*transpose([e]))*cos(angle) - skew(e)*sin(angle)
                = identity(3) - skew(e)*angle, for small angles
                = identity(3) - skew(e*angle)
                   define phi = e*angle, then
           Trel = [1,      phi3,   -phi2;
                   -phi3,     1,    phi1;
                    phi2, -phi1,       1 ];
      */
  phi := if withResidues then {R.T[2, 3], -R.T[1, 3], R.T[1, 2], R.T[1, 1] - 1, R.T[2, 2] - 1, R.T[1, 1] * R.T[2, 2] - R.T[2, 1] * R.T[1, 2] - 1} else {R.T[2, 3], -R.T[1, 3], R.T[1, 2]};
  annotation(Inline = true);
end smallRotation;
