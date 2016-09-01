package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.octoMap.node.AbstractOccupancyOcTreeNode;

public class AbstractOccupancyOcTreeNodeMessage<N extends AbstractOccupancyOcTreeNodeMessage<N, O>, O extends AbstractOccupancyOcTreeNode<O>> extends AbstractOcTreeNodeMessage<N, O>
{
   public float logOdds;

}
