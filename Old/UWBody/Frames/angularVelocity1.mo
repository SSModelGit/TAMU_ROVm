within UWBody.Frames;

function angularVelocity1 "Return angular velocity resolved in frame 1 from orientation object"
  extends Modelica.Icons.Function;
  input Orientation R "Orientation object to rotate frame 1 into frame 2";
  output Modelica.SIunits.AngularVelocity w[3] "Angular velocity of frame 2 with respect to frame 1 resolved in frame 1";
algorithm
  w := resolve1(R, R.w);
  annotation(Inline = true, Documentation(info = "<html>
<h4>Syntax</h4>
<blockquote><pre>
w12_1 = Frames.<b>angularVelocity1</b>(R12);
</pre></blockquote>

<h4>Description</h4>
<p>
The function call <code>Frames.<b>angularVelocity1</b>(R12)</code> returns the
the angular velocity w12_1 of frame 2 with respect to frame 1 resolved in frame 1,
from the orientation object R12 that describes the orientation to rotate frame 1 into frame 2.
</p>
</html>"));
end angularVelocity1;
