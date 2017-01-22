package us.ihmc.robotEnvironmentAwareness.communication.packets;

import java.util.Arrays;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import us.ihmc.communication.packets.Packet;
import us.ihmc.jOctoMap.key.OcTreeKey;

public class PlanarRegionSegmentationMessage extends Packet<PlanarRegionSegmentationMessage>
{
   public int id;
   public Point3f origin;
   public Vector3f normal;
   public OcTreeKeyMessage[] nodeKeys;
   public Point3f[] hitLocations;

   public PlanarRegionSegmentationMessage()
   {
   }

   public PlanarRegionSegmentationMessage(int id, Point3d origin, Vector3d normal, OcTreeKeyMessage[] regionNodeKeys, List<Point3d> hitLocations)
   {
      this.id = id;
      this.origin = new Point3f(origin);
      this.normal = new Vector3f(normal);
      this.nodeKeys = regionNodeKeys;
      this.hitLocations = hitLocations.stream().map(Point3f::new).toArray(Point3f[]::new);
   }

   public PlanarRegionSegmentationMessage(int id, Point3f origin, Vector3f normal, OcTreeKeyMessage[] regionNodeKeys, Point3f[] hitLocations)
   {
      this.id = id;
      this.origin = origin;
      this.normal = normal;
      this.nodeKeys = regionNodeKeys;
      this.hitLocations = hitLocations;
   }

   public int getRegionId()
   {
      return id;
   }

   public Point3f getOrigin()
   {
      return origin;
   }

   public Vector3f getNormal()
   {
      return normal;
   }

   public int getNumberOfNodes()
   {
      return nodeKeys.length;
   }

   public OcTreeKeyMessage getNodeKey(int index)
   {
      return nodeKeys[index];
   }

   public Point3f[] getHitLocations()
   {
      return hitLocations;
   }

   public Point3f getHitLocation(int index)
   {
      return hitLocations[index];
   }

   public void getNodeKey(int index, OcTreeKey nodeKeyToPack)
   {
      nodeKeyToPack.set(nodeKeys[index]);
   }

   public void getHitLocation(int index, Point3d hitLocationToPack)
   {
      hitLocationToPack.set(hitLocations[index]);
   }

   @Override
   public boolean epsilonEquals(PlanarRegionSegmentationMessage other, double epsilon)
   {
      if (id != other.id)
         return false;
      if (!origin.epsilonEquals(other.origin, (float) epsilon))
         return false;
      if (!normal.epsilonEquals(other.normal, (float) epsilon))
         return false;
      if (!Arrays.equals(nodeKeys, other.nodeKeys))
         return false;
      if (hitLocations.length != other.hitLocations.length)
         return false;
      for (int i = 0; i < hitLocations.length; i++)
      {
         if (!hitLocations[i].epsilonEquals(other.hitLocations[i], (float) epsilon))
            return false;
      }
      return true;
   }
}
