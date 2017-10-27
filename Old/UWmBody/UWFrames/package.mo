within UWmBody;

package UWFrames "Functions to transform rotational frame quantities"
  extends Modelica.Icons.Package;
  import Forces = UWmBody.UWForces;
  import Interfaces = UWmBody.UWInterfaces;
  import Joints = UWmBody.UWJoints;
  import Parts = UWmBody.UWParts;
  import Sensors = UWmBody.UWSensors;
  import Visualizers = UWmBody.Visualizers;
  import Types = UWmBody.UWTypes;
  import Icons = UWmBody.UWIcons;
  annotation(Documentation(info = "<html>
<p>
Package <b>Frames</b> contains type definitions and
functions to transform rotational frame quantities. The basic idea is to
hide the actual definition of an <b>orientation</b> in this package
by providing essentially type <b>Orientation</b> together with
<b>functions</b> operating on instances of this type.
</p>
<h4>Content</h4>
<p>In the table below an example is given for every function definition.
The used variables have the following declaration:
</p>
<pre>
   Frames.Orientation R, R1, R2, R_rel, R_inv;
   Real[3,3]   T, T_inv;
   Real[3]     v1, v2, w1, w2, n_x, n_y, n_z, e, e_x, res_ori, phi;
   Real[6]     res_equal;
   Real        L, angle;
</pre>
<table border=1 cellspacing=0 cellpadding=2>
  <tr><th><b><i>Function/type</i></b></th><th><b><i>Description</i></b></th></tr>
  <tr><td valign=\"top\"><b>Orientation R;</b></td>
      <td valign=\"top\">New type defining an orientation object that describes<br>
          the rotation of frame 1 into frame 2.
      </td>
  </tr>
  <tr><td valign=\"top\">res_ori = <b>orientationConstraint</b>(R);</td>
      <td valign=\"top\">Return the constraints between the variables of an orientation object<br>
      (shall be zero).</td>
  </tr>
  <tr><td valign=\"top\">w1 = <b>angularVelocity1</b>(R);</td>
      <td valign=\"top\">Return angular velocity resolved in frame 1 from
          orientation object R.
     </td>
  </tr>
  <tr><td valign=\"top\">w2 = <b>angularVelocity2</b>(R);</td>
      <td valign=\"top\">Return angular velocity resolved in frame 2 from
          orientation object R.
     </td>
  </tr>
  <tr><td valign=\"top\">v1 = <b>resolve1</b>(R,v2);</td>
      <td valign=\"top\">Transform vector v2 from frame 2 to frame 1.
      </td>
  </tr>
  <tr><td valign=\"top\">v2 = <b>resolve2</b>(R,v1);</td>
      <td valign=\"top\">Transform vector v1 from frame 1 to frame 2.
     </td>
  </tr>
  <tr><td valign=\"top\">v2 = <b>resolveRelative</b>(v1,R1,R2);</td>
      <td valign=\"top\">Transform vector v1 from frame 1 to frame 2
          using absolute orientation objects R1 of frame 1 and R2 of frame 2.
      </td>
  </tr>
  <tr><td valign=\"top\">D1 = <b>resolveDyade1</b>(R,D2);</td>
      <td valign=\"top\">Transform second order tensor D2 from frame 2 to frame 1.
      </td>
  </tr>
  <tr><td valign=\"top\">D2 = <b>resolveDyade2</b>(R,D1);</td>
      <td valign=\"top\">Transform second order tensor D1 from frame 1 to frame 2.
     </td>
  </tr>
  <tr><td valign=\"top\">R = <b>nullRotation</b>()</td>
      <td valign=\"top\">Return orientation object R that does not rotate a frame.
     </td>
  </tr>
  <tr><td valign=\"top\">R_inv = <b>inverseRotation</b>(R);</td>
      <td valign=\"top\">Return inverse orientation object.
      </td>
  </tr>
  <tr><td valign=\"top\">R_rel = <b>relativeRotation</b>(R1,R2);</td>
      <td valign=\"top\">Return relative orientation object from two absolute
          orientation objects.
      </td>
  </tr>
  <tr><td valign=\"top\">R2 = <b>absoluteRotation</b>(R1,R_rel);</td>
      <td valign=\"top\">Return absolute orientation object from another
          absolute<br> and a relative orientation object.
      </td>
  </tr>
  <tr><td valign=\"top\">R = <b>planarRotation</b>(e, angle, der_angle);</td>
      <td valign=\"top\">Return orientation object of a planar rotation.
      </td>
  </tr>
  <tr><td valign=\"top\">angle = <b>planarRotationAngle</b>(e, v1, v2);</td>
      <td valign=\"top\">Return angle of a planar rotation, given the rotation axis<br>
        and the representations of a vector in frame 1 and frame 2.
      </td>
  </tr>
  <tr><td valign=\"top\">R = <b>axisRotation</b>(axis, angle, der_angle);</td>
      <td valign=\"top\">Return orientation object R to rotate around angle along axis of frame 1.
      </td>
  </tr>
  <tr><td valign=\"top\">R = <b>axesRotations</b>(sequence, angles, der_angles);</td>
      <td valign=\"top\">Return rotation object to rotate in sequence around 3 axes. Example:<br>
          R = axesRotations({1,2,3},{pi/2,pi/4,-pi}, zeros(3));
      </td>
  </tr>
  <tr><td valign=\"top\">angles = <b>axesRotationsAngles</b>(R, sequence);</td>
      <td valign=\"top\">Return the 3 angles to rotate in sequence around 3 axes to<br>
          construct the given orientation object.
      </td>
  </tr>
  <tr><td valign=\"top\">phi = <b>smallRotation</b>(R);</td>
      <td valign=\"top\">Return rotation angles phi valid for a small rotation R.
      </td>
  </tr>
  <tr><td valign=\"top\">R = <b>from_nxy</b>(n_x, n_y);</td>
      <td valign=\"top\">Return orientation object from n_x and n_y vectors.
      </td>
  </tr>
  <tr><td valign=\"top\">R = <b>from_nxz</b>(n_x, n_z);</td>
      <td valign=\"top\">Return orientation object from n_x and n_z vectors.
      </td>
  </tr>
  <tr><td valign=\"top\">R = <b>from_T</b>(T,w);</td>
      <td valign=\"top\">Return orientation object R from transformation matrix T and
          its angular velocity w.
      </td>
  </tr>
  <tr><td valign=\"top\">R = <b>from_T2</b>(T,der(T));</td>
      <td valign=\"top\">Return orientation object R from transformation matrix T and
          its derivative der(T).
      </td>
  </tr>
  <tr><td valign=\"top\">R = <b>from_T_inv</b>(T_inv,w);</td>
      <td valign=\"top\">Return orientation object R from inverse transformation matrix T_inv and
          its angular velocity w.
      </td>
  </tr>
  <tr><td valign=\"top\">R = <b>from_Q</b>(Q,w);</td>
      <td valign=\"top\">Return orientation object R from quaternion orientation object Q
          and its angular velocity w.
      </td>
  </tr>
  <tr><td valign=\"top\">T = <b>to_T</b>(R);</td>
      <td valign=\"top\">Return transformation matrix T from orientation object R.
      </td>
  </tr>
  <tr><td valign=\"top\">T_inv = <b>to_T_inv</b>(R);</td>
      <td valign=\"top\">Return inverse transformation matrix T_inv from orientation object R.
      </td>
  </tr>
  <tr><td valign=\"top\">Q = <b>to_Q</b>(R);</td>
      <td valign=\"top\">Return quaternion orientation object Q from orientation object R.
      </td>
  </tr>
  <tr><td valign=\"top\">exy = <b>to_exy</b>(R);</td>
      <td valign=\"top\">Return [e_x, e_y] matrix of an orientation object R, <br>
          with e_x and e_y vectors of frame 2, resolved in frame 1.
      </td>
  </tr>
  <tr><td valign=\"top\">L = <b>length</b>(n_x);</td>
      <td valign=\"top\">Return length L of a vector n_x.
      </td>
  </tr>
  <tr><td valign=\"top\">e_x = <b>normalize</b>(n_x);</td>
      <td valign=\"top\">Return normalized vector e_x of n_x such that length of e_x is one.
      </td>
  </tr>
  <tr><td valign=\"top\">e = <b>axis</b>(i);</td>
      <td valign=\"top\">Return unit vector e directed along axis i
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Frames.Quaternions\">Quaternions</a></td>
      <td valign=\"top\"><b>Package</b> with functions to transform rotational frame quantities based
          on quaternions (also called Euler parameters).
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Frames.TransformationMatrices\">TransformationMatrices</a></td>
      <td valign=\"top\"><b>Package</b> with functions to transform rotational frame quantities based
          on transformation matrices.
      </td>
  </tr>
</table>
</html>"), Icon(graphics = {Line(points = {{-2, -18}, {80, -60}}, color = {95, 95, 95}), Line(points = {{-2, -18}, {-2, 80}}, color = {95, 95, 95}), Line(points = {{-78, -56}, {-2, -18}}, color = {95, 95, 95})}));
end UWFrames;
