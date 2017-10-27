within RBodyInFluid.Fields.Internal;

function standardBuoyantForce
  extends RBodyInFluid.Interfaces.partialBuoyantForce;
  input Modelica.SIunits.Density pho_fluid "Density of fluid surrounding submerged object";
  input Modelica.SIunits.Acceleration g[3] "Constant gravity acceleration value, resolved in world frame, opposite to direction of actual gravity" annotation(Dialog);
protected
  final parameter Modelica.SIunits.RelativeDensity d_star = pho_fluid / d;
algorithm
  buoyant_a := -d_star * m * g;
end standardBuoyantForce;
