package us.ihmc.robotEnvironmentAwareness.communication.packets;

import us.ihmc.jOctoMap.node.NormalOcTreeNode;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

/**
 * Created by adrien on 11/21/16.
 */
public class OctreeNodeData
{
//   private float normalX = Float.NaN;
//   private float normalY = Float.NaN;
//   private float normalZ = Float.NaN;
//   private float hitLocationX = Float.NaN;
//   private float hitLocationY = Float.NaN;
//   private float hitLocationZ = Float.NaN;
//   private float size = Float.NaN;
   private final NormalOcTreeNode normalOcTreeNode;
   private final int regionId;

   public OctreeNodeData()
   {
      normalOcTreeNode = null;
      regionId = Integer.MIN_VALUE;
   }

   public OctreeNodeData(NormalOcTreeNode normalOcTreeNodeToCopy, int regionId)
   {
      this.normalOcTreeNode = new NormalOcTreeNode();
      this.normalOcTreeNode.copyData(normalOcTreeNodeToCopy);

      this.regionId = regionId;
   }


   public NormalOcTreeNode getNormalOcTreeNode()
   {
      return normalOcTreeNode;
   }


//   public OctreeNodeData(float normalX, float normalY, float normalZ, float hitLocationX, float hitLocationY, float hitLocationZ, float size, int regionId)
//   {
//      this.normalX = normalX;
//      this.normalY = normalY;
//      this.normalZ = normalZ;
//      this.hitLocationX = hitLocationX;
//      this.hitLocationY = hitLocationY;
//      this.hitLocationZ = hitLocationZ;
//      this.size = size;
//      this.regionId = regionId;
//   }

//   public boolean isHitLocationSet()
//   {
//      return !Float.isNaN(hitLocationX) && !Float.isNaN(hitLocationY) && !Float.isNaN(hitLocationY);
//   }
//
//   public boolean isNormalSet()
//   {
//      return !Float.isNaN(normalX) && !Float.isNaN(normalY) && !Float.isNaN(normalZ);
//   }
//
//   public void getNormal(Vector3d normalToPack)
//   {
//      normalToPack.set(normalX, normalY, normalZ);
//   }
//
//   public float getNormalX()
//   {
//      return normalX;
//   }
//
//   public float getNormalY()
//   {
//      return normalY;
//   }
//
//   public float getNormalZ()
//   {
//      return normalZ;
//   }
//
//   public float getHitLocationX()
//   {
//      return hitLocationX;
//   }
//
//   public float getHitLocationY()
//   {
//      return hitLocationY;
//   }
//
//   public float getHitLocationZ()
//   {
//      return hitLocationZ;
//   }
//
//   public float getSize()
//   {
//      return size;
//   }


//   public void getHitLocation(Point3d hitLocationToPack)
//   {
//      hitLocationToPack.set(hitLocationX, hitLocationY, hitLocationZ);
//   }

   public int getRegionId()
   {
      return regionId;
   }
}
