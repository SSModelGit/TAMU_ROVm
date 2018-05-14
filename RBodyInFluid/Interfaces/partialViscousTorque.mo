within RBodyInFluid.Interfaces;

partial function partialViscousTorque "Torque drag due to viscous forces"
  extends Modelica.Icons.Function;
  input Modelica.SIunits.AngularVelocity w[:];
  output Modelica.SIunits.Torque t_d[:];
end partialViscousTorque;
