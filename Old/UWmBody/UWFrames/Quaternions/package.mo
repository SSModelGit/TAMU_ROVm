within UWmBody.UWFrames;

package Quaternions "Functions to transform rotational frame quantities based on quaternions (also called Euler parameters)"
  extends Modelica.Icons.Package;
  annotation(Documentation(info = "<html>
<p>
Package <b>Frames.Quaternions</b> contains type definitions and
functions to transform rotational frame quantities with quaternions.
Functions of this package are currently only utilized in
MultiBody.Parts.Body components, when quaternions shall be used
as parts of the body states.
Some functions are also used in a new Modelica package for
B-Spline interpolation that is able to interpolate paths consisting of
position vectors and orientation objects.
</p>
<h4>Content</h4>
<p>In the table below an example is given for every function definition.
The used variables have the following declaration:
</p>
<pre>
   Quaternions.Orientation Q, Q1, Q2, Q_rel, Q_inv;
   Real[3,3]   T, T_inv;
   Real[3]     v1, v2, w1, w2, n_x, n_y, n_z, res_ori, phi;
   Real[6]     res_equal;
   Real        L, angle;
</pre>
<table border=1 cellspacing=0 cellpadding=2>
  <tr><th><b><i>Function/type</i></b></th><th><b><i>Description</i></b></th></tr>
  <tr><td valign=\"top\"><b>Orientation Q;</b></td>
      <td valign=\"top\">New type defining a quaternion object that describes<br>
          the rotation of frame 1 into frame 2.
      </td>
  </tr>
  <tr><td valign=\"top\"><b>der_Orientation</b> der_Q;</td>
      <td valign=\"top\">New type defining the first time derivative
         of Frames.Quaternions.Orientation.
      </td>
  </tr>
  <tr><td valign=\"top\">res_ori = <b>orientationConstraint</b>(Q);</td>
      <td valign=\"top\">Return the constraints between the variables of a quaternion object<br>
      (shall be zero).</td>
  </tr>
  <tr><td valign=\"top\">w1 = <b>angularVelocity1</b>(Q, der_Q);</td>
      <td valign=\"top\">Return angular velocity resolved in frame 1 from
          quaternion object Q<br> and its derivative der_Q.
     </td>
  </tr>
  <tr><td valign=\"top\">w2 = <b>angularVelocity2</b>(Q, der_Q);</td>
      <td valign=\"top\">Return angular velocity resolved in frame 2 from
          quaternion object Q<br> and its derivative der_Q.
     </td>
  </tr>
  <tr><td valign=\"top\">v1 = <b>resolve1</b>(Q,v2);</td>
      <td valign=\"top\">Transform vector v2 from frame 2 to frame 1.
      </td>
  </tr>
  <tr><td valign=\"top\">v2 = <b>resolve2</b>(Q,v1);</td>
      <td valign=\"top\">Transform vector v1 from frame 1 to frame 2.
     </td>
  </tr>
  <tr><td valign=\"top\">[v1,w1] = <b>multipleResolve1</b>(Q, [v2,w2]);</td>
      <td valign=\"top\">Transform several vectors from frame 2 to frame 1.
      </td>
  </tr>
  <tr><td valign=\"top\">[v2,w2] = <b>multipleResolve2</b>(Q, [v1,w1]);</td>
      <td valign=\"top\">Transform several vectors from frame 1 to frame 2.
      </td>
  </tr>
  <tr><td valign=\"top\">Q = <b>nullRotation</b>()</td>
      <td valign=\"top\">Return quaternion object R that does not rotate a frame.
      </td>
  </tr>
  <tr><td valign=\"top\">Q_inv = <b>inverseRotation</b>(Q);</td>
      <td valign=\"top\">Return inverse quaternion object.
      </td>
  </tr>
  <tr><td valign=\"top\">Q_rel = <b>relativeRotation</b>(Q1,Q2);</td>
      <td valign=\"top\">Return relative quaternion object from two absolute
          quaternion objects.
      </td>
  </tr>
  <tr><td valign=\"top\">Q2 = <b>absoluteRotation</b>(Q1,Q_rel);</td>
      <td valign=\"top\">Return absolute quaternion object from another
          absolute<br> and a relative quaternion object.
      </td>
  </tr>
  <tr><td valign=\"top\">Q = <b>planarRotation</b>(e, angle);</td>
      <td valign=\"top\">Return quaternion object of a planar rotation.
      </td>
  </tr>
  <tr><td valign=\"top\">phi = <b>smallRotation</b>(Q);</td>
      <td valign=\"top\">Return rotation angles phi valid for a small rotation.
      </td>
  </tr>
  <tr><td valign=\"top\">Q = <b>from_T</b>(T);</td>
      <td valign=\"top\">Return quaternion object Q from transformation matrix T.
      </td>
  </tr>
  <tr><td valign=\"top\">Q = <b>from_T_inv</b>(T_inv);</td>
      <td valign=\"top\">Return quaternion object Q from inverse transformation matrix T_inv.
      </td>
  </tr>
  <tr><td valign=\"top\">T = <b>to_T</b>(Q);</td>
      <td valign=\"top\">Return transformation matrix T from quaternion object Q.
      </td>
  </tr>
  <tr><td valign=\"top\">T_inv = <b>to_T_inv</b>(Q);</td>
      <td valign=\"top\">Return inverse transformation matrix T_inv from quaternion object Q.
      </td>
  </tr>
</table>
</html>"));
end Quaternions;
