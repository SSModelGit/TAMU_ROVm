within UWBody.Frames;

function resolveDyade1 "Transform second order tensor from frame 2 to frame 1"
  extends Modelica.Icons.Function;
  input Orientation R "Orientation object to rotate frame 1 into frame 2";
  input Real D2[3, 3] "Second order tensor resolved in frame 2";
  output Real D1[3, 3] "Second order tensor resolved in frame 1";
algorithm
  D1 := transpose(R.T) * D2 * R.T;
  annotation(Inline = true, Documentation(info = "<html>
<h4>Syntax</h4>
<blockquote><pre>
D1 = Frames.<b>resolveDyade1</b>(R12, D2);
</pre></blockquote>

<h4>Description</h4>
<p>
The function call <code>Frames.<b>Dyade1</b>(R12, D2)</code> returns the second order tensor D
resolved in frame 1 (= D1) from its representation in frame 2 (= D2) using the
orientation object R12 that describes the orientation to rotate frame 1 into frame 2.
</p>
</html>"));
end resolveDyade1;
