package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.octoMap.node.AbstractOcTreeNode;

public class AbstractOcTreeBaseMessage<N extends AbstractOcTreeNodeMessage<N, O>, O extends AbstractOcTreeNode<O>>
{
   public double resolution;
   public N rootNode;

   public AbstractOcTreeBaseMessage()
   {
   }

   public double getResolution()
   {
      return resolution;
   }

   public void setResolution(double resolution)
   {
      this.resolution = resolution;
   }
}
