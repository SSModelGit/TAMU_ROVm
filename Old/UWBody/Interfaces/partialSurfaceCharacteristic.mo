within UWBody.Interfaces;

partial function partialSurfaceCharacteristic
  extends Modelica.Icons.Function;
  input Integer nu "Number of points in u-Dimension";
  input Integer nv "Number of points in v-Dimension";
  input Boolean multiColoredSurface = false "= true: Color is defined for each surface point";
  output Modelica.SIunits.Position X[nu, nv] "[nu,nv] positions of points in x-Direction resolved in surface frame";
  output Modelica.SIunits.Position Y[nu, nv] "[nu,nv] positions of points in y-Direction resolved in surface frame";
  output Modelica.SIunits.Position Z[nu, nv] "[nu,nv] positions of points in z-Direction resolved in surface frame";
  output Real C[if multiColoredSurface then nu else 0, if multiColoredSurface then nv else 0, 3] "[nu,nv,3] Color array, defining the color for each surface point";
end partialSurfaceCharacteristic;
