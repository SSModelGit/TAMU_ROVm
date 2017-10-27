within UWmBody.UWFrames;

function angularVelocity2 "Return angular velocity resolved in frame 2 from orientation object"
  extends Modelica.Icons.Function;
  input Orientation R "Orientation object to rotate frame 1 into frame 2";
  output Modelica.SIunits.AngularVelocity w[3] "Angular velocity of frame 2 with respect to frame 1 resolved in frame 2";
algorithm
  w := R.w;
  annotation(Inline = true, Documentation(info = "<html>
<h4>Syntax</h4>
<blockquote><pre>
w12_2 = Frames.<b>angularVelocity2</b>(R12);
</pre></blockquote>

<h4>Description</h4>
<p>
The function call <code>Frames.<b>angularVelocity2</b>(R12)</code> returns the
the angular velocity w12_2 of frame 2 with respect to frame 1 resolved in frame 2,
from the orientation object R12 that describes the orientation to rotate frame 1 into frame 2.
</p>
</html>"));
end angularVelocity2;
