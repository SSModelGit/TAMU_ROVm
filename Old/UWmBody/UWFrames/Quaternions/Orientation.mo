within UWmBody.UWFrames.Quaternions;

type Orientation "Orientation type defining rotation from a frame 1 into a frame 2 with quaternions {p1,p2,p3,p0}"
  extends Internal.QuaternionBase;

  encapsulated function equalityConstraint "Return the constraint residues to express that two frames have the same quaternion orientation"
    import Modelica;
    import UWmBody.UWFrames.Quaternions;
    extends Modelica.Icons.Function;
    input Quaternions.Orientation Q1 "Quaternions orientation object to rotate frame 0 into frame 1";
    input Quaternions.Orientation Q2 "Quaternions orientation object to rotate frame 0 into frame 2";
    output Real residue[3] "Zero vector if Q1 and Q2 are identical (the first three elements of the relative transformation (is {0,0,0} for the null rotation, guarded by atan2 to make the mirrored solution invalid";
  algorithm
    residue := {atan2({Q1[4], Q1[3], -Q1[2], -Q1[1]} * Q2, Q1 * Q2), atan2({-Q1[3], Q1[4], Q1[1], -Q1[2]} * Q2, Q1 * Q2), atan2({Q1[2], -Q1[1], Q1[4], -Q1[3]} * Q2, Q1 * Q2)};
    annotation(Inline = true);
  end equalityConstraint;
  annotation(Documentation(info = "<html>
<p>
This type describes the <b>rotation</b> to rotate a frame 1 into
a frame 2 using quaternions (also called <b>Euler parameters</b>)
according to the following definition:
</p>
<pre>
   Quaternions.Orientation Q;
   Real  n[3];
   Real  phi(unit=\"rad\");
   Q = [ n*sin(phi/2)
           cos(phi/2) ]
</pre>
<p>
where \"n\" is the <b>axis of rotation</b> to rotate frame 1 into
frame 2 and \"phi\" is the <b>rotation angle</b> for this rotation.
Vector \"n\" is either resolved in frame 1 or in frame 2
(the result is the same since the coordinates of \"n\" with respect to
frame 1 are identical to its coordinates with respect to frame 2).
</p>
<p>
The term \"quaternions\" is preferred over the historically
more reasonable \"Euler parameters\" in order to not get
confused with Modelica \"parameters\".
</p>
</html>"));
end Orientation;
