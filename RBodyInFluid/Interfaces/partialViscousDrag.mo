within RBodyInFluid.Interfaces;

partial function partialViscousDrag
  extends Modelica.Icons.Function;
  input Modelica.SIunits.Velocity v[:];
  output Modelica.SIunits.Force f_d[:];
end partialViscousDrag;
