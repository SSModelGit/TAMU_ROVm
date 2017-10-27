within UWmBody;

package UWJoints "Components that constrain the motion between two frames"
  extends Modelica.Icons.Package;
  import Forces = UWmBody.UWForces;
  import Interfaces = UWmBody.UWInterfaces;
  import Frames = UWmBody.UWFrames;
  import Parts = UWmBody.UWParts;
  import Sensors = UWmBody.UWSensors;
  import Visualizers = UWmBody.Visualizers;
  import Types = UWmBody.UWTypes;
  import Icons = UWmBody.UWIcons;
  annotation(Documentation(info = "<html>
<p>
This package contains <b>joint components</b>,
that is, idealized, massless elements that constrain
the motion between frames. In subpackage <b>Assemblies</b>
aggregation joint components are provided to handle
kinematic loops analytically (this means that non-linear systems
of equations occurring in these joint aggregations are analytically
solved, i.e., robustly and efficiently).
</p>
<h4>Content</h4>
<table border=1 cellspacing=0 cellpadding=2>
  <tr><th><b><i>Model</i></b></th><th><b><i>Description</i></b></th></tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Prismatic\">Prismatic</a>
      <td valign=\"top\">Prismatic joint and actuated prismatic joint
          (1 translational degree-of-freedom, 2 potential states)<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/Prismatic.png\">
      </td></td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Revolute\">Revolute</a>
 </td>
      <td valign=\"top\">Revolute and actuated revolute joint
          (1 rotational degree-of-freedom, 2 potential states)<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/Revolute.png\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Cylindrical\">Cylindrical</a></td>
      <td valign=\"top\">Cylindrical joint (2 degrees-of-freedom, 4 potential states)<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/Cylindrical.png\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Universal\">Universal</a></td>
      <td valign=\"top\">Universal joint (2 degrees-of-freedom, 4 potential states)<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/Universal.png\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Planar\">Planar</a></td>
      <td valign=\"top\">Planar joint (3 degrees-of-freedom, 6 potential states)<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/Planar.png\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Spherical\">Spherical</a></td>
      <td valign=\"top\">Spherical joint (3 constraints and no potential states, or 3 degrees-of-freedom and 3 states)<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/Spherical.png\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.FreeMotion\">FreeMotion</a></td>
      <td valign=\"top\">Free motion joint (6 degrees-of-freedom, 12 potential states)<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/FreeMotion.png\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.SphericalSpherical\">SphericalSpherical</a></td>
      <td valign=\"top\">Spherical - spherical joint aggregation (1 constraint,
          no potential states) with an optional point mass in the middle<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/SphericalSpherical.png\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.UniversalSpherical\">UniversalSpherical</a></td>
      <td valign=\"top\">Universal - spherical joint aggregation (1 constraint, no potential states)<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Joints/UniversalSpherical.png\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.GearConstraint\">GearConstraint</a></td>
      <td valign=\"top\">Ideal 3-dim. gearbox (arbitrary shaft directions)
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Assemblies\">MultiBody.Joints.Assemblies</a></td>
      <td valign=\"top\"><b>Package</b> of joint aggregations for analytic loop handling.
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Constraints\">MultiBody.Joints.Constraints</a></td>
      <td valign=\"top\"><b>Package</b> of components that define joints by constraints
      </td>
  </tr>
</table>
</html>"), Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, origin = {35, -20}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.VerticalCylinder, extent = {{-15, 0}, {15, 105}}), Rectangle(visible = true, origin = {35, -20}, rotation = 120, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.VerticalCylinder, extent = {{-15, 0}, {15, 105}}), Ellipse(visible = true, origin = {35, -20}, rotation = -45, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-20, -20}, {20, 20}}), Ellipse(visible = true, origin = {35, -20}, lineColor = {255, 255, 255}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, extent = {{-10, -10}, {10, 10}})}));
end UWJoints;
