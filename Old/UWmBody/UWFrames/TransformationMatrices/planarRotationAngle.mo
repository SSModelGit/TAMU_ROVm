within UWmBody.UWFrames.TransformationMatrices;

function planarRotationAngle "Return angle of a planar rotation, given the rotation axis and the representations of a vector in frame 1 and frame 2"
  extends Modelica.Icons.Function;
  input Real e[3](each final unit = "1") "Normalized axis of rotation to rotate frame 1 around e into frame 2 (must have length=1)";
  input Real v1[3] "A vector v resolved in frame 1 (shall not be parallel to e)";
  input Real v2[3] "Vector v resolved in frame 2, i.e., v2 = resolve2(planarRotation(e,angle),v1)";
  output Modelica.SIunits.Angle angle "Rotation angle to rotate frame 1 into frame 2 along axis e in the range: -pi <= angle <= pi";
algorithm
  /* Vector v is resolved in frame 1 and frame 2 according to:
              (1)  v2 = (e*transpose(e) + (identity(3) - e*transpose(e))*cos(angle) - skew(e)*sin(angle))*v1;
                      = e*(e*v1) + (v1 - e*(e*v1))*cos(angle) - cross(e,v1)*sin(angle)
             Equation (1) is multiplied with "v1" resulting in (note: e*e = 1)
                  v1*v2 = (e*v1)*(e*v2) + (v1*v1 - (e*v1)*(e*v1))*cos(angle)
             and therefore:
              (2) cos(angle) = ( v1*v2 - (e*v1)*(e*v2)) / (v1*v1 - (e*v1)*(e*v1))
             Similarly, equation (1) is multiplied with cross(e,v1), i.e., a
             a vector that is orthogonal to e and to v1:
                    cross(e,v1)*v2 = - cross(e,v1)*cross(e,v1)*sin(angle)
             and therefore:
                (3) sin(angle) = -cross(e,v1)*v2/(cross(e,v1)*cross(e,v1));
             We have e*e=1; Therefore:
                (4) v1*v1 - (e*v1)*(e*v1) = |v1|^2 - (|v1|*cos(e,v1))^2
             and
                (5) cross(e,v1)*cross(e,v1) = (|v1|*sin(e,v1))^2
                                            = |v1|^2*(1 - cos(e,v1)^2)
                                            = |v1|^2 - (|v1|*cos(e,v1))^2
             The denominators of (2) and (3) are identical, according to (4) and (5).
             Furthermore, the denominators are always positive according to (5).
             Therefore, in the equation "angle = atan2(sin(angle), cos(angle))" the
             denominators of sin(angle) and cos(angle) can be removed,
             resulting in:
                angle = atan2(-cross(e,v1)*v2, v1*v2 - (e*v1)*(e*v2));
          */
  angle := Modelica.Math.atan2(-cross(e, v1) * v2, v1 * v2 - e * v1 * (e * v2));
  annotation(Inline = true, Documentation(info = "<html>
<p>
A call to this function of the form
</p>
<pre>
    Real[3]                e, v1, v2;
    Modelica.SIunits.Angle angle;
  <b>equation</b>
    angle = <b>planarRotationAngle</b>(e, v1, v2);
</pre>
<p>
computes the rotation angle \"<b>angle</b>\" of a planar
rotation along unit vector <b>e</b>, rotating frame 1 into frame 2, given
the coordinate representations of a vector \"v\" in frame 1 (<b>v1</b>)
and in frame 2 (<b>v2</b>). Therefore, the result of this function
fulfills the following equation:
</p>
<pre>
    v2 = <b>resolve2</b>(<b>planarRotation</b>(e,angle), v1)
</pre>
<p>
The rotation angle is returned in the range
</p>
<pre>
    -<font face=\"Symbol\">p</font> &lt;= angle &lt;= <font face=\"Symbol\">p</font>
</pre>
<p>
This function makes the following assumptions on the input arguments
</p>
<ul>
<li> Vector <b>e</b> has length 1, i.e., length(e) = 1</li>
<li> Vector \"v\" is not parallel to <b>e</b>, i.e.,
     length(cross(e,v1)) &ne; 0</li>
</ul>
<p>
The function does not check the above assumptions. If these
assumptions are violated, a wrong result will be returned
and/or a division by zero will occur.
</p>
</html>"));
end planarRotationAngle;
