package us.ihmc.robotEnvironmentAwareness.communication;

import java.util.List;

import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeNodeMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.OcTreeNodePlanarRegion;

public class OcTreeMessageConverter
{

   public static NormalOcTreeMessage convertToMessage(NormalOcTree normalOcTree)
   {
      return convertToMessage(normalOcTree, null);
   }

   public static NormalOcTreeMessage convertToMessage(NormalOcTree normalOcTree, List<OcTreeNodePlanarRegion> planarRegions)
   {
      NormalOcTreeMessage normalOcTreeMessage = new NormalOcTreeMessage();

      normalOcTreeMessage.resolution = (float) normalOcTree.getResolution();
      normalOcTreeMessage.treeDepth = normalOcTree.getTreeDepth();

      if (normalOcTree.getRoot() != null)
      {
         normalOcTreeMessage.root = new NormalOcTreeNodeMessage();
         fullDepthCopy(normalOcTree.getRoot(), planarRegions, normalOcTreeMessage.root);
      }

      return normalOcTreeMessage;
   }

   private static void fullDepthCopy(NormalOcTreeNode nodeOriginal, List<OcTreeNodePlanarRegion> planarRegions, NormalOcTreeNodeMessage nodeCopy)
   {
      if (planarRegions != null)
         planarRegions.parallelStream().filter(region -> region.contains(nodeOriginal)).findFirst().ifPresent(region -> nodeCopy.regionId = region.getId());

      nodeCopy.depth = nodeOriginal.getDepth();
      nodeCopy.size = (float) nodeOriginal.getSize();

      nodeCopy.key0 = nodeOriginal.getKey0();
      nodeCopy.key1 = nodeOriginal.getKey1();
      nodeCopy.key2 = nodeOriginal.getKey2();

      nodeCopy.normalX = (float) nodeOriginal.getNormalX();
      nodeCopy.normalY = (float) nodeOriginal.getNormalY();
      nodeCopy.normalZ = (float) nodeOriginal.getNormalZ();

      nodeCopy.normalAverageDeviation = nodeCopy.normalAverageDeviation;
      nodeCopy.normalConsensusSize = nodeCopy.normalConsensusSize;

      nodeCopy.hitLocationX = (float) nodeOriginal.getHitLocationX();
      nodeCopy.hitLocationY = (float) nodeOriginal.getHitLocationY();
      nodeCopy.hitLocationZ = (float) nodeOriginal.getHitLocationZ();

      nodeCopy.numberOfHits = nodeOriginal.getNumberOfHits();

      if (nodeOriginal.hasAtLeastOneChild())
      {
         nodeCopy.children = new NormalOcTreeNodeMessage[8];
         for (int childIndex = 0; childIndex < 8; childIndex++)
         {
            NormalOcTreeNode childOriginal = nodeOriginal.getChild(childIndex);
            if (childOriginal == null)
               continue;

            NormalOcTreeNodeMessage childCopy = new NormalOcTreeNodeMessage();
            nodeCopy.children[childIndex] = childCopy;
            fullDepthCopy(childOriginal, planarRegions, childCopy);
         }
      }
   }
}
