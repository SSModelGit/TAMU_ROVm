within UWBody.Types;

type ResolveInFrameB = enumeration(world "Resolve in world frame", frame_b "Resolve in frame_b", frame_resolve "Resolve in frame_resolve (frame_resolve must be connected)") "Enumeration to define the frame in which an absolute vector is resolved (world, frame_b, frame_resolve)" annotation(Documentation(info = "<html>
   <table border=1 cellspacing=0 cellpadding=2>
   <tr><th><b>Types.ResolveInFrameB.</b></th><th><b>Meaning</b></th></tr>
   <tr><td valign=\"top\">world</td>
       <td valign=\"top\">Resolve vector in world frame</td></tr>
   
   <tr><td valign=\"top\">frame_b</td>
       <td valign=\"top\">Resolve vector in frame_b</td></tr>
   
   <tr><td valign=\"top\">frame_resolve</td>
       <td valign=\"top\">Resolve vector in frame_resolve (frame_resolve must be connected)</td></tr>
   </table>
   </html>"));
