within UWmBody.UWJoints;

package Assemblies "Components that aggregate several joints for analytic loop handling"
  extends Modelica.Icons.Package;
  annotation(Documentation(info = "<html>
<p>
The joints in this package are mainly designed to be used
in <b>kinematic loop</b> structures. Every component consists of
<b>3 elementary joints</b>. These joints are combined in such a
way that the kinematics of the 3 joints between frame_a and
frame_b are computed from the movement of frame_a and frame_b,
i.e., there are <b>no constraints</b> between frame_a and frame_b.
This requires to solve a <b>non-linear system of equations</b> which
is performed <b>analytically</b> (i.e., when a mathematical
solution exists, it is computed efficiently and reliably).
A detailed description how to use these joints is provided in
<a href=\"modelica://Modelica.Mechanics.MultiBody.UsersGuide.Tutorial.LoopStructures.AnalyticLoopHandling\">MultiBody.UsersGuide.Tutorial.LoopStructures.AnalyticLoopHandling</a>.
</p>
<p>
The assembly joints in this package are named <b>JointXYZ</b> where
<b>XYZ</b> are the first letters of the elementary joints used in the
component, in particular:
</p>
<table border=1 cellspacing=0 cellpadding=2>
  <tr><td valign=\"top\"><b>P</b></td><td valign=\"top\">Prismatic joint</td></tr>
  <tr><td valign=\"top\"><b>R</b></td><td valign=\"top\">Revolute joint</td></tr>
  <tr><td valign=\"top\"><b>S</b></td><td valign=\"top\">Spherical joint</td></tr>
  <tr><td valign=\"top\"><b>U</b></td><td valign=\"top\">Universal joint</td></tr>
</table>
<p>
For example, JointUSR is an assembly joint consisting
of a universal, a spherical and a revolute joint.
</p>
<p> This package contains the following models:
</p>
<h4>Content</h4>
<table border=1 cellspacing=0 cellpadding=2>
  <tr><th><b><i>Model</i></b></th><th><b><i>Description</i></b></th></tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Assemblies.JointUPS\">JointUPS</a></td>
      <td valign=\"top\"> Universal - prismatic - spherical joint aggregation<br>
     <img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/JointUPS.png\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Assemblies.JointUSR\">JointUSR</a></td>
      <td valign=\"top\"> Universal - spherical - revolute joint aggregation<br>
     <img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/JointUSR.png\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Assemblies.JointUSP\">JointUSP</a></td>
      <td valign=\"top\"> Universal - spherical - prismatic joint aggregation<br>
     <img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/JointUSP.png\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Assemblies.JointSSR\">JointSSR</a></td>
      <td valign=\"top\"> Spherical - spherical - revolute joint aggregation
           with an optional mass point at the rod connecting
           the two spherical joints<br>
     <img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/JointSSR.png\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Assemblies.JointSSP\">JointSSP</a></td>
      <td valign=\"top\"> Spherical - spherical - prismatic joint aggregation
           with an optional mass point at the rod connecting
           the two spherical joints<br>
     <img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/JointSSP.png\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Assemblies.JointRRR\">JointRRR</a></td>
      <td valign=\"top\"> Revolute - revolute - revolute joint aggregation for planar loops<br>
     <img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/JointRRR.png\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Assemblies.JointRRP\">JointRRP</a></td>
      <td valign=\"top\"> Revolute - revolute - prismatic joint aggregation for planar loops<br>
     <img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/JointRRP.png\">
      </td>
  </tr>
</table>
<p>
Note, no component of this package has potential states, since the
components are designed in such a way that the generalized coordinates
of the used elementary joints are computed from the frame_a and frame_b
coordinates. Still, it is possible to use the components in a
tree structure. In this case states are selected from bodies that are
connected to the frame_a or frame_b side of the component.
In most cases this gives a less efficient solution, as if elementary
joints of package Modelica.Mechanics.MultiBody.Joints would be used directly.
</p>
<p>
The analytic handling of kinematic loops by using joint aggregations
with 6 degrees of freedom as provided in this package, is a <b>new</b>
methodology. It is based on a more general method for solving
non-linear equations of kinematic loops developed by Woernle and
Hiller. An automatic application of this more general method
is difficult, and a manual application is only suited for
specialists in this field. The method introduced here is a
compromise: It can be quite easily applied by an end user, but
for a smaller class of kinematic loops. The method of the \"characteristic
pair of joints\" from Woernle and Hiller is described in:
</p>
<dl>
<dt>Woernle C.:</dt>
<dd><b>Ein systematisches Verfahren zur Aufstellung der geometrischen
    Schliessbedingungen in kinematischen Schleifen mit Anwendung
    bei der R&uuml;ckw&auml;rtstransformation f&uuml;r
    Industrieroboter.</b><br>
    Fortschritt-Berichte VDI, Reihe 18, Nr. 59, Duesseldorf: VDI-Verlag 1988,
    ISBN 3-18-145918-6.<br>&nbsp;</dd>
<dt>Hiller M., and Woernle C.:</dt>
<dd><b>A Systematic Approach for Solving the Inverse Kinematic
    Problem of Robot Manipulators</b>.<br>
    Proceedings 7th World Congress Th. Mach. Mech., Sevilla 1987. </dd>
</dl>
</html>"));
end Assemblies;
