within UWmBody.UWFrames.TransformationMatrices;

function from_Q "Return orientation object T from quaternion orientation object Q"
  extends Modelica.Icons.Function;
  input Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
  output TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
algorithm
  /*
        T := (2*Q[4]*Q[4] - 1)*identity(3) + 2*([Q[1:3]]*transpose([Q[1:3]]) - Q[4]*
          skew(Q[1:3]));
      */
  T := [2 * (Q[1] * Q[1] + Q[4] * Q[4]) - 1, 2 * (Q[1] * Q[2] + Q[3] * Q[4]), 2 * (Q[1] * Q[3] - Q[2] * Q[4]); 2 * (Q[2] * Q[1] - Q[3] * Q[4]), 2 * (Q[2] * Q[2] + Q[4] * Q[4]) - 1, 2 * (Q[2] * Q[3] + Q[1] * Q[4]); 2 * (Q[3] * Q[1] + Q[2] * Q[4]), 2 * (Q[3] * Q[2] - Q[1] * Q[4]), 2 * (Q[3] * Q[3] + Q[4] * Q[4]) - 1];
  annotation(Inline = true);
end from_Q;
