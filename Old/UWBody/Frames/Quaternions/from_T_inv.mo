within UWBody.Frames.Quaternions;

function from_T_inv "Return quaternion orientation object Q from inverse transformation matrix T_inv"
  extends Modelica.Icons.Function;
  input Real T_inv[3, 3] "Inverse transformation matrix to transform vector from frame 2 to frame 1 (v1=T_inv*v2)";
  input Quaternions.Orientation Q_guess = nullRotation() "Guess value for output Q (there are 2 solutions; the one closer to Q_guess is used";
  output Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2 (Q and -Q have same transformation matrix)";
algorithm
  Q := from_T(transpose(T_inv), Q_guess);
  annotation(Inline = true);
end from_T_inv;
