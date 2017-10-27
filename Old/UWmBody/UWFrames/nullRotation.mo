within UWmBody.UWFrames;

function nullRotation "Return orientation object that does not rotate a frame"
  extends Modelica.Icons.Function;
  output Orientation R "Orientation object such that frame 1 and frame 2 are identical";
algorithm
  R := Orientation(T = identity(3), w = zeros(3));
  annotation(Inline = true, Documentation(info = "<html>
<h4>Syntax</h4>
<blockquote><pre>
R = Frames.<b>nullRotation</b>(R);
</pre></blockquote>

<h4>Description</h4>
<p>
The function call <code>Frames.<b>rnullRotation</b>()</code> returns an orientation matrix
R describing the orientation object to rotate frame 1 into frame 2, if frame 1 and frame 2 are identical.
(= transformation matrix is identity matrix and angular velocity is zero).
</p>
</html>"));
end nullRotation;
