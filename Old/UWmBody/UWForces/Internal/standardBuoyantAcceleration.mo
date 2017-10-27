within UWmBody.UWForces.Internal;

function standardBuoyantAcceleration "Standard buoyant force field (parallel to gravity)"
  extends Modelica.Icons.Function;
  extends UWmBody.UWInterfaces.partialBuoyantAcceleration;
  input Modelica.SIunits.Density pho_fluid "Density of fluid surrounding submerged object";
  input Modelica.SIunits.Acceleration g_a "Constant gravity acceleration value, resolved in world frame, opposite to direction of actual gravity" annotation(Dialog);
  input UWTypes.Axis dir annotation(Dialog);
protected
  final parameter Modelica.SIunits.RelativeDensity d_star = pho_fluid / d;
algorithm
  buoyant_a := -d_star * g_a * Modelica.Math.Vectors.normalizeWithAssert(dir);
end standardBuoyantAcceleration;
