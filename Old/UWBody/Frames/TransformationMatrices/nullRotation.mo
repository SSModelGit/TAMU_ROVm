within UWBody.Frames.TransformationMatrices;

function nullRotation "Return orientation object that does not rotate a frame"
  extends Modelica.Icons.Function;
  output TransformationMatrices.Orientation T "Orientation object such that frame 1 and frame 2 are identical";
algorithm
  T := identity(3);
  annotation(Inline = true);
end nullRotation;
