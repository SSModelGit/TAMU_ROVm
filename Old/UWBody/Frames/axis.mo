within UWBody.Frames;

function axis "Return unit vector for x-, y-, or z-axis"
  extends Modelica.Icons.Function;
  input Integer axis(min = 1, max = 3) "Axis vector to be returned";
  output Real e[3](each final unit = "1") "Unit axis vector";
algorithm
  e := if axis == 1 then {1, 0, 0} else if axis == 2 then {0, 1, 0} else {0, 0, 1};
  annotation(Inline = true);
end axis;
