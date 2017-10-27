within UWmBody.UWFrames.TransformationMatrices;

function axisRotation "Return rotation object to rotate around one frame axis"
  extends Modelica.Icons.Function;
  input Integer axis(min = 1, max = 3) "Rotate around 'axis' of frame 1";
  input Modelica.SIunits.Angle angle "Rotation angle to rotate frame 1 into frame 2 along 'axis' of frame 1";
  output TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
algorithm
  T := if axis == 1 then [1, 0, 0; 0, cos(angle), sin(angle); 0, -sin(angle), cos(angle)] else if axis == 2 then [cos(angle), 0, -sin(angle); 0, 1, 0; sin(angle), 0, cos(angle)] else [cos(angle), sin(angle), 0; -sin(angle), cos(angle), 0; 0, 0, 1];
  annotation(Inline = true);
end axisRotation;
