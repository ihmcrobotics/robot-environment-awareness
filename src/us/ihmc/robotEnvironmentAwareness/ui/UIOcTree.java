package us.ihmc.robotEnvironmentAwareness.ui;

import us.ihmc.jOctoMap.ocTree.baseImplementation.AbstractOcTreeBase;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;

public class UIOcTree extends AbstractOcTreeBase<UIOcTreeNode>
{
   public UIOcTree(NormalOcTreeMessage normalOcTreeMessage)
   {
      super(normalOcTreeMessage.resolution, normalOcTreeMessage.treeDepth);
      root = new UIOcTreeNode(normalOcTreeMessage.root, resolution, treeDepth);
   }

   @Override
   protected Class<UIOcTreeNode> getNodeClass()
   {
      return UIOcTreeNode.class;
   }
}
