within UWmBody.UWForces.Internal;

function standardHydroDynamicDrag "Hydrodynamic drag = c * v^2"
  extends Modelica.Icons.Function;
  extends UWmBody.UWInterfaces.partialDragForce;
  input Modelica.SIunits.DimensionlessRatio c annotation(Dialog);
  input Modelica.SIunits.Area A annotation(Dialog);
  input Modelica.SIunits.Density pho_fluid annotation(Dialog);
algorithm
  // f_d := -0.5 * pho_fluid * c * A * Modelica.Math.Vectors.length(v) * v * 0.01;
  f_d := -0.5 * pho_fluid * c * A * v;
end standardHydroDynamicDrag;
