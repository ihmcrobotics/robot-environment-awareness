package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.octoMap.node.AbstractOccupancyOcTreeNode;

public class AbstractOccupancyOcTreeMessage<N extends AbstractOccupancyOcTreeNodeMessage<N, O>, O extends AbstractOccupancyOcTreeNode<O>> extends AbstractOcTreeBaseMessage<N, O>
{
   public float minOccupancyLogOdds;
   public float maxOccupancyLogOdds;
   public float hitUpdateLogOdds;
   public float missUpdateLogOdds;
   public float occupancyThresholdLogOdds;
}
