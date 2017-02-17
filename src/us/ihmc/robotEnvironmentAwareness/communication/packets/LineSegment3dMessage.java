package us.ihmc.robotEnvironmentAwareness.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.robotics.geometry.LineSegment3d;

public class LineSegment3dMessage extends Packet<LineSegment3dMessage>
{
   public Point3D32 start, end;

   public LineSegment3dMessage()
   {
   }

   public LineSegment3dMessage(Point3D32 start, Point3D32 end)
   {
      this.start = start;
      this.end = end;
   }

   public LineSegment3dMessage(LineSegment3d lineSegment3d)
   {
      start = new Point3D32(lineSegment3d.getFirstEndpoint());
      end = new Point3D32(lineSegment3d.getSecondEndpoint());
   }

   public Point3D32 getStart()
   {
      return start;
   }

   public Point3D32 getEnd()
   {
      return end;
   }

   public void setStart(Point3D32 start)
   {
      this.start = start;
   }

   public void setEnd(Point3D32 end)
   {
      this.end = end;
   }

   @Override
   public boolean epsilonEquals(LineSegment3dMessage other, double epsilon)
   {
      return start.epsilonEquals(other.start, (float) epsilon) && end.epsilonEquals(other.end, (float) epsilon);
   }
}
