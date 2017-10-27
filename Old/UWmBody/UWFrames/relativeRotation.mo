within UWmBody.UWFrames;

function relativeRotation "Return relative orientation object"
  extends Modelica.Icons.Function;
  input Orientation R1 "Orientation object to rotate frame 0 into frame 1";
  input Orientation R2 "Orientation object to rotate frame 0 into frame 2";
  output Orientation R_rel "Orientation object to rotate frame 1 into frame 2";
algorithm
  R_rel := Orientation(T = R2.T * transpose(R1.T), w = R2.w - resolve2(R2, resolve1(R1, R1.w)));
  annotation(Inline = true, Documentation(info = "<html>
<h4>Syntax</h4>
<blockquote><pre>
R_rel = Frames.<b>relativeRotation</b>(R1,R2);
</pre></blockquote>

<h4>Description</h4>
<p>
The function call <code>Frames.<b>relativeRotation</b>(R1,R2)</code> returns
orientation object R_rel that describes the orientation to rotate frame 1 to frame 2
from the orientation object R1 that describes the orientation to rotate from frame 0 to frame 1 and
from the orientation object R2 that describes the orientation to rotate from frame 0 to frame 2.
</p>
</html>"));
end relativeRotation;
