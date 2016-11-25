package us.ihmc.robotEnvironmentAwareness.ui;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import boofcv.misc.UnsupportedException;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOccupancyOcTreeNode;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeNodeMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.OcTreeNodePlanarRegion;

public class UIOcTreeNode extends AbstractOccupancyOcTreeNode<UIOcTreeNode>
{
   private final int regionId;
   private final float normalX;
   private final float normalY;
   private final float normalZ;
   private final float normalAverageDeviation;
   private final int normalConsensusSize;
   private final float hitLocationX;
   private final float hitLocationY;
   private final float hitLocationZ;

   private final long numberOfHits;

   public UIOcTreeNode(NormalOcTreeNodeMessage normalOcTreeNodeMessage, double resolution, int treeDepth)
   {
      int k0 = normalOcTreeNodeMessage.key0;
      int k1 = normalOcTreeNodeMessage.key1;
      int k2 = normalOcTreeNodeMessage.key2;
      int depth = normalOcTreeNodeMessage.depth;
      setProperties(k0, k1, k2, depth, resolution, treeDepth);

      regionId = normalOcTreeNodeMessage.regionId;
      normalX = normalOcTreeNodeMessage.normalX;
      normalY = normalOcTreeNodeMessage.normalY;
      normalZ = normalOcTreeNodeMessage.normalZ;
      normalAverageDeviation = normalOcTreeNodeMessage.normalAverageDeviation;
      normalConsensusSize = normalOcTreeNodeMessage.normalConsensusSize;
      hitLocationX = normalOcTreeNodeMessage.hitLocationX;
      hitLocationY = normalOcTreeNodeMessage.hitLocationY;
      hitLocationZ = normalOcTreeNodeMessage.hitLocationZ;
      numberOfHits = normalOcTreeNodeMessage.numberOfHits;

      if (normalOcTreeNodeMessage.getNumberOfChildren() > 0)
      {
         super.allocateChildren();

         for (int childIndex = 0; childIndex < 8; childIndex++)
         {
            NormalOcTreeNodeMessage otherChild = normalOcTreeNodeMessage.children[childIndex];
            if (otherChild != null)
            {
               children[childIndex] = new UIOcTreeNode(otherChild, resolution, treeDepth);
            }
         }
      }
   }

   public boolean isNormalSet()
   {
      return !Float.isNaN(normalX) && !Float.isNaN(normalY) && !Float.isNaN(normalZ);
   }

   public boolean isHitLocationSet()
   {
      return !Float.isNaN(hitLocationX) && !Float.isNaN(hitLocationY) && !Float.isNaN(hitLocationZ);
   }

   public boolean isPartOfRegion()
   {
      return regionId != OcTreeNodePlanarRegion.NO_REGION_ID;
   }

   public void getNormal(Vector3d normalToPack)
   {
      normalToPack.set(normalX, normalY, normalZ);
   }

   public float getNormalAverageDeviation()
   {
      return normalAverageDeviation;
   }

   public int getNormalConsensusSize()
   {
      return normalConsensusSize;
   }

   public void getHitLocation(Point3d hitLocationToPack)
   {
      hitLocationToPack.set(hitLocationX, hitLocationY, hitLocationZ);
   }

   public long getNumberOfHits()
   {
      return numberOfHits;
   }

   public int getRegionId()
   {
      return regionId;
   }

   @Override
   protected void clear()
   {
      throw new UnsupportedException();
   }

   @Override
   public void addValue(float logOdds)
   {
      throw new UnsupportedException();
   }

   @Override
   public void allocateChildren()
   {
      throw new UnsupportedException();
   }
}
