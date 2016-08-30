package us.ihmc.robotEnvironmentAwareness.simulation;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.Packet;

public class LidarPosePacket extends Packet<LidarPosePacket>
{
   public Point3d position;
   public Quat4d orientation;
   public float lidarJointAngle;

   public LidarPosePacket()
   {
   }

   public LidarPosePacket(Point3d position, Quat4d orientation)
   {
      this.position = position;
      this.orientation = orientation;
   }

   public LidarPosePacket(LidarPosePacket other)
   {
      if (other.position != null)
         position = new Point3d(other.position);
      else
         position = null;
      if (other.orientation != null)
         orientation = new Quat4d(other.orientation);
      else
         orientation = null;
      lidarJointAngle = other.lidarJointAngle;
   }

   public void setLidarAngleJoint(float jointAngle)
   {
      lidarJointAngle = jointAngle;
   }

   public Point3d getPosition()
   {
      return position;
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }

   public float getLidarJointAngle()
   {
      return lidarJointAngle;
   }

   @Override
   public boolean epsilonEquals(LidarPosePacket other, double epsilon)
   {
      return false;
   }
}
