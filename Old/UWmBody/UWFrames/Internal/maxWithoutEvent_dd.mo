within UWmBody.UWFrames.Internal;

function maxWithoutEvent_dd "First derivative of function maxWithoutEvent_d(..)"
  extends Modelica.Icons.Function;
  input Real u1;
  input Real u2;
  input Real u1_d;
  input Real u2_d;
  input Real u1_dd;
  input Real u2_dd;
  output Real y_dd;
algorithm
  y_dd := if u1 > u2 then u1_dd else u2_dd;
  annotation(Inline = true);
end maxWithoutEvent_dd;
