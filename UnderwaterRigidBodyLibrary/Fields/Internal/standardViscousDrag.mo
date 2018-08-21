within UnderwaterRigidBodyLibrary.Fields.Internal;

function standardViscousDrag
  extends UnderwaterRigidBodyLibrary.Interfaces.partialViscousDrag;
  input Modelica.SIunits.DimensionlessRatio mu annotation(Dialog);
  input Modelica.SIunits.Area A annotation(Dialog);
algorithm
  f_d := -1 * mu * A * v;
end standardViscousDrag;
