within UWmBody.UWFrames.TransformationMatrices;

function angularVelocity1 "Return angular velocity resolved in frame 1 from orientation object and its derivative"
  extends Modelica.Icons.Function;
  input TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
  input der_Orientation der_T "Derivative of T";
  output Modelica.SIunits.AngularVelocity w[3] "Angular velocity of frame 2 with respect to frame 1 resolved in frame 1";
algorithm
  /* The angular velocity w of frame 2 with respect to frame 1 resolved in frame 1,
           is defined as:
              w = vec( der(transpose(T))*T );
           where
                         |   0 -w3  w2 |
               skew(w) = |  w3   0 -w1 | and w = vec(skew(w))
                         | -w2  w1   0 |
           i.e.
               W = der(transpose(T))*T)
               w = {W(3,2), -W(3,1), W(2,1)}
           Therefore, only 3 values of W need to be computed:
                   | der(T[:,1]) |
               W = | der(T[:,2]) | * | T[:,1], T[:,2], T[:,3] |
                   | der(T[:,3]) |
                   |  W(3,2) |   |  der(T[:,3])*T[:,2] |
               w = | -W(3,1) | = | -der(T[:,3])*T[:,1] |
                   |  W(2,1) |   |  der(T[:,2])*T[:,1] |
        */
  w := {der_T[:, 3] * T[:, 2], -der_T[:, 3] * T[:, 1], der_T[:, 2] * T[:, 1]};
  annotation(Inline = true);
end angularVelocity1;
