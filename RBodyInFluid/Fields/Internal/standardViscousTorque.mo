within RBodyInFluid.Fields.Internal;

function standardViscousTorque "Torque drag due to viscous forces"
  extends RBodyInFluid.Interfaces.partialViscousTorque;
  input Modelica.SIunits.RotationalDampingConstant k;
algorithm
  t_d := -k * w;
end standardViscousTorque;
