within UWBody.Forces;

model SpringDamperSeries "Linear spring and linear damper in series connection"
  parameter SI.TranslationalSpringConstant c(final min = 0) "Spring constant";
  parameter SI.Length s_unstretched = 0 "Unstretched spring length";
  parameter SI.TranslationalDampingConstant d(final min = 0) = 0 "Damping constant";
  parameter SI.Length s_damper_start = 0 "Initial length of damper";
  SI.Position s_damper(start = s_damper_start, fixed = true) "Actual length of damper (frame_a - damper - spring - frame_b)";
  extends Interfaces.PartialLineForce;
  extends Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPort(final T = 293.15);
equation
  f = c * (s - s_unstretched - s_damper);
  d * der(s_damper) = f;
  lossPower = f * der(s_damper);
  annotation(Documentation(info = "<html>
<p>
<b>Linear spring</b> and <b>linear damper</b> in series connection
acting as line force between frame_a and frame_b:
</p>
<pre>
  frame_a --> damper ----> spring --> frame_b
          |              |
          |-- s_damper --|  (s_damper is the state variable of this system)
</pre>
<p>
A <b>force f</b> is exerted on the origin of frame_b and with opposite sign
on the origin of frame_a along the line from the origin of frame_a to the origin
of frame_b according to the equations:
</p>
<pre>
   f = c*(s - s_unstretched - s_damper);
   f = d*der(s_damper);
</pre>
<p>
where \"c\", \"s_unstretched\" and \"d\" are parameters, \"s\" is the
distance between the origin of frame_a and the origin of frame_b.
\"s_damper\" is the length of the damper (= an internal state of this
force element) and der(s_damper) is the time derivative of s_damper.
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{-100, 0}, {-15, 0}}, color = {64, 64, 64}), Line(visible = true, points = {{-60, -30}, {-15, -30}}, color = {64, 64, 64}), Line(visible = true, points = {{-60, 30}, {-15, 30}}, color = {64, 64, 64}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-60, -30}, {-30, 30}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 50}, {150, 90}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -65}, {150, -35}}, textString = "c=%c"), Line(visible = useHeatPort, points = {{-100, -99}, {-100, -24}, {-45, -24}}, color = {191, 0, 0}, pattern = LinePattern.Dot), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -100}, {150, -70}}, textString = "d=%d"), Polygon(visible = true, origin = {15, 0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, points = {{-30.185, 4}, {-12, 4}, {-6, -20}, {10, 34}, {18, 34}, {34, -20}, {48, 34}, {56, 34}, {66, 4}, {85, 4}, {85, -4}, {60, -4}, {52, 20}, {38, -34}, {30, -34}, {14, 20}, {-2, -34}, {-10, -34}, {-18, -4}, {-30.185, -4}}), Polygon(visible = true, origin = {15, 0}, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{-12, 4}, {-6, -20}, {-10, -34}, {-18, -4}}), Polygon(visible = true, origin = {15, 0}, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{14, 20}, {18, 34}, {34, -20}, {30, -34}}), Polygon(visible = true, origin = {15, 0}, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{52, 20}, {56, 34}, {66, 4}, {60, -4}}), Polygon(visible = true, origin = {15, 0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, points = {{-30.461, 4}, {-12, 4}, {-18, -4}, {-30.461, -4}}), Polygon(visible = true, origin = {15, 0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, points = {{66, 4}, {85, 4}, {85, -4}, {60, -4}})}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{-100, 0}, {-15, 0}}, color = {64, 64, 64}), Line(visible = true, points = {{-60, -30}, {-15, -30}}, color = {64, 64, 64}), Line(visible = true, points = {{-60, 30}, {-15, 30}}, color = {64, 64, 64}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-60, -30}, {-30, 30}}), Line(visible = true, points = {{-75, 0}, {-75, 85}}, color = {128, 128, 128}), Line(visible = true, points = {{-10, 0}, {-10, 65}}, color = {128, 128, 128}), Line(visible = true, points = {{80, 0}, {80, 85}}, color = {128, 128, 128}), Line(visible = true, points = {{-75, 80}, {80, 80}}, color = {128, 128, 128}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 15), Line(visible = true, origin = {-0, -10}, points = {{-75, 60}, {-10, 60}}, color = {128, 128, 128}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 15), Text(visible = true, origin = {1.671, -8}, textColor = {128, 128, 128}, extent = {{-76.389, 63}, {-11.671, 78}}, textString = "s_damper"), Text(visible = true, textColor = {128, 128, 128}, extent = {{0, 80}, {20, 100}}, textString = "s"), Polygon(visible = true, origin = {15, -0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, points = {{-30.185, 4}, {-12, 4}, {-6, -20}, {10, 34}, {18, 34}, {34, -20}, {48, 34}, {56, 34}, {66, 4}, {85, 4}, {85, -4}, {60, -4}, {52, 20}, {38, -34}, {30, -34}, {14, 20}, {-2, -34}, {-10, -34}, {-18, -4}, {-30.185, -4}}), Polygon(visible = true, origin = {15, -0}, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{-12, 4}, {-6, -20}, {-10, -34}, {-18, -4}}), Polygon(visible = true, origin = {15, -0}, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{14, 20}, {18, 34}, {34, -20}, {30, -34}}), Polygon(visible = true, origin = {15, -0}, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{52, 20}, {56, 34}, {66, 4}, {60, -4}}), Polygon(visible = true, origin = {15, -0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, points = {{-30.461, 4}, {-12, 4}, {-18, -4}, {-30.461, -4}}), Polygon(visible = true, origin = {15, -0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, points = {{66, 4}, {85, 4}, {85, -4}, {60, -4}})}));
end SpringDamperSeries;
