package us.ihmc.robotEnvironmentAwareness.communication.packets;

import javax.vecmath.Point3f;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.geometry.LineSegment3d;

public class LineSegment3dMessage extends Packet<LineSegment3dMessage>
{
   public Point3f start, end;

   public LineSegment3dMessage()
   {
   }

   public LineSegment3dMessage(Point3f start, Point3f end)
   {
      this.start = start;
      this.end = end;
   }

   public LineSegment3dMessage(LineSegment3d lineSegment3d)
   {
      start = new Point3f(lineSegment3d.getFirstEndpoint());
      end = new Point3f(lineSegment3d.getSecondEndpoint());
   }

   public Point3f getStart()
   {
      return start;
   }

   public Point3f getEnd()
   {
      return end;
   }

   public void setStart(Point3f start)
   {
      this.start = start;
   }

   public void setEnd(Point3f end)
   {
      this.end = end;
   }

   @Override
   public boolean epsilonEquals(LineSegment3dMessage other, double epsilon)
   {
      return start.epsilonEquals(other.start, (float) epsilon) && end.epsilonEquals(other.end, (float) epsilon);
   }
}
