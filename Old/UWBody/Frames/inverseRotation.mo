within UWBody.Frames;

function inverseRotation "Return inverse orientation object"
  extends Modelica.Icons.Function;
  input Orientation R "Orientation object to rotate frame 1 into frame 2";
  output Orientation R_inv "Orientation object to rotate frame 2 into frame 1";
algorithm
  R_inv := Orientation(T = transpose(R.T), w = -resolve1(R, R.w));
  annotation(Inline = true, Documentation(info = "<html>
<h4>Syntax</h4>
<blockquote><pre>
R_inv = Frames.<b>inverseRotation</b>(R);
</pre></blockquote>

<h4>Description</h4>
<p>
The function call <code>Frames.<b>inverseRotation</b>(R)</code> returns
orientation object R_inv that describes the orientation to rotate from frame 2 to frame 1
from the orientation object R that describes the orientation to rotate from frame 1 into frame 2.
</p>
</html>"));
end inverseRotation;
