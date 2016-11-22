package us.ihmc.robotEnvironmentAwareness.communication.packets;

/**
 * Created by adrien on 11/21/16.
 */
public class OctreeNodeData
{
   private float normalX = Float.NaN;
   private float normalY = Float.NaN;
   private float normalZ = Float.NaN;
   private float hitLocationX = Float.NaN;
   private float hitLocationY = Float.NaN;
   private float hitLocationZ = Float.NaN;
   private float size = Float.NaN;
   private int regionId = Integer.MIN_VALUE;

   public OctreeNodeData()
   {

   }

   public OctreeNodeData(float normalX, float normalY, float normalZ, float hitLocationX, float hitLocationY, float hitLocationZ, float size, int regionId)
   {
      this.normalX = normalX;
      this.normalY = normalY;
      this.normalZ = normalZ;
      this.hitLocationX = hitLocationX;
      this.hitLocationY = hitLocationY;
      this.hitLocationZ = hitLocationZ;
      this.size = size;
      this.regionId = regionId;
   }



   public float getNormalX()
   {
      return normalX;
   }

   public float getNormalY()
   {
      return normalY;
   }

   public float getNormalZ()
   {
      return normalZ;
   }

   public float getHitLocationX()
   {
      return hitLocationX;
   }

   public float getHitLocationY()
   {
      return hitLocationY;
   }

   public float getHitLocationZ()
   {
      return hitLocationZ;
   }

   public float getSize()
   {
      return size;
   }

   public int getRegionId()
   {
      return regionId;
   }
}
