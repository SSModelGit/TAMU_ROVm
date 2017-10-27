within UWmBody.UWJoints;

package Constraints "Components that define joints by constraints"
  extends Modelica.Icons.Package;
  annotation(Documentation(info = "<html>
<p>
This package contains <b>constraint components</b>, that is, idealized, massless elements that
constrain the motion between frames by means of kinematic constraints. The constraint
elements are especially aimed to be used for multibody models which contain <b>kinematic loops</b>.
Usually, kinematic loops are automatically handled. However, the performance might be improved
by either solving certain kinds of loops analytically with the help of the components of
subpackage  <a href=\"Modelica://Modelica.Mechanics.MultiBody.Joints.Assemblies\">Assemblies</a>, or
by providing numerically better loop constraint formulations with the help of the components
of this subpackage.
</p>
</html>"));
end Constraints;
