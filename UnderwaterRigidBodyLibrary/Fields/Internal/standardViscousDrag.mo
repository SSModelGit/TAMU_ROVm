within UnderwaterRigidBodyLibrary.Fields.Internal;

function standardViscousDrag
  extends UnderwaterRigidBodyLibrary.Interfaces.partialViscousDrag;
  input Modelica.SIunits.DimensionlessRatio c annotation(Dialog);
  input Modelica.SIunits.Area A annotation(Dialog);
  input Modelica.SIunits.Density pho_fluid annotation(Dialog);
algorithm
  // f_d := -0.5 * pho_fluid * c * A * v * Modelica.Math.Vectors.length(v);
  f_d := -0.5 * pho_fluid * c * A * v;
end standardViscousDrag;