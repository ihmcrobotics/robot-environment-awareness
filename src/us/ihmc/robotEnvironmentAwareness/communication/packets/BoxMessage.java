package us.ihmc.robotEnvironmentAwareness.communication.packets;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import us.ihmc.communication.packets.Packet;

public class BoxMessage extends Packet<BoxMessage>
{
   public boolean isEmpty;
   public Vector3f size;
   public Point3f center;
   public Quat4f orientation;

   public BoxMessage()
   {
   }

   public static BoxMessage emptyBox()
   {
      BoxMessage empty = new BoxMessage();
      empty.isEmpty = true;
      empty.size = null;
      empty.center = null;
      empty.orientation = null;
      return empty;
   }

   public boolean isEmpty()
   {
      return isEmpty;
   }

   public void setSize(Vector3d size)
   {
      isEmpty = false;
      this.size = new Vector3f(size);
   }

   public void setSize(Vector3f size)
   {
      isEmpty = false;
      this.size = size;
   }

   public void setCenter(Point3d center)
   {
      isEmpty = false;
      this.center = new Point3f(center);
   }

   public void setCenter(Point3f center)
   {
      isEmpty = false;
      this.center = center;
   }

   public void setOrientation(Quat4d orientation)
   {
      isEmpty = false;
      this.orientation = new Quat4f(orientation);
   }

   public void setOrientation(Quat4f orientation)
   {
      isEmpty = false;
      this.orientation = orientation;
   }

   public Vector3f getSize()
   {
      return size;
   }

   public Point3f getCenter()
   {
      return center;
   }

   public Quat4f getOrientation()
   {
      return orientation;
   }

   @Override
   public boolean epsilonEquals(BoxMessage other, double epsilon)
   {
      if (!size.epsilonEquals(other.size, (float) epsilon))
         return false;
      if (!center.epsilonEquals(other.center, (float) epsilon))
         return false;
      if (!orientation.epsilonEquals(other.orientation, (float) epsilon))
         return false;
      return true;
   }
}
