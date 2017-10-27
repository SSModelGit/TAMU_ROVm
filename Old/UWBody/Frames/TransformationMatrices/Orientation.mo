within UWBody.Frames.TransformationMatrices;

type Orientation "Orientation type defining rotation from a frame 1 into a frame 2 with a transformation matrix"
  extends Internal.TransformationMatrix;

  encapsulated function equalityConstraint "Return the constraint residues to express that two frames have the same orientation"
    import Modelica;
    import UWBody.Frames.TransformationMatrices;
    extends Modelica.Icons.Function;
    input TransformationMatrices.Orientation T1 "Orientation object to rotate frame 0 into frame 1";
    input TransformationMatrices.Orientation T2 "Orientation object to rotate frame 0 into frame 2";
    output Real residue[3] "The rotation angles around x-, y-, and z-axis of frame 1 to rotate frame 1 into frame 2 for a small rotation (should be zero)";
  algorithm
    residue := {cross(T1[1, :], T1[2, :]) * T2[2, :], -cross(T1[1, :], T1[2, :]) * T2[1, :], T1[2, :] * T2[1, :]};
    annotation(Inline = true);
  end equalityConstraint;
  annotation(Documentation(info = "<html>
<p>
This type describes the <b>rotation</b> from a <b>frame 1</b> into a <b>frame 2</b>.
An instance <b>R</b> of type <b>Orientation</b> has the following interpretation:
</p>
<pre>
   <b>T</b> = [<b>e</b><sub>x</sub>, <b>e</b><sub>y</sub>, <b>e</b><sub>z</sub>];
       e.g., <b>T</b> = [1,0,0; 0,1,0; 0,0,1]
</pre>
<p>
where <b>e</b><sub>x</sub>,<b>e</b><sub>y</sub>,<b>e</b><sub>z</sub>
are unit vectors in the direction of the x-axis, y-axis, and z-axis
of frame 1, resolved in frame 2, respectively. Therefore, if <b>v</b><sub>1</sub>
is vector <b>v</b> resolved in frame 1 and <b>v</b><sub>2</sub> is
vector <b>v</b> resolved in frame 2, the following relationship holds:
</p>
<pre>
    <b>v</b><sub>2</sub> = <b>T</b> * <b>v</b><sub>1</sub>
</pre>
<p>
The <b>inverse</b> orientation
<b>T_inv</b> = <b>T</b><sup>T</sup> describes the rotation
from frame 2 into frame 1.
</p>
<p>
Since the orientation is described by 9 variables, there are
6 constraints between these variables. These constraints
are defined in function <b>TransformationMatrices.orientationConstraint</b>.
</p>
<p>
Note, that in the MultiBody library the rotation object is
never directly accessed but only with the access functions provided
in package TransformationMatrices. As a consequence, other implementations of
Rotation can be defined by adapting this package correspondingly.
</p>
</html>"));
end Orientation;
