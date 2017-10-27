within UWmBody.UWFrames.Quaternions;

function angularVelocity1 "Compute angular velocity resolved in frame 1 from quaternion orientation object and its derivative"
  extends Modelica.Icons.Function;
  input Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
  input der_Orientation der_Q "Derivative of Q";
  output Modelica.SIunits.AngularVelocity w[3] "Angular velocity resolved in frame 1";
algorithm
  w := 2 * ([Q[4], -Q[3], Q[2], -Q[1]; Q[3], Q[4], -Q[1], -Q[2]; -Q[2], Q[1], Q[4], -Q[3]] * der_Q);
  annotation(Inline = true);
end angularVelocity1;
