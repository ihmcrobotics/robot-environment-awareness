package us.ihmc.robotEnvironmentAwareness.communication;

import javax.vecmath.Vector3f;

import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.ocTree.PlanarRegion;

public class NormalOcTreeNodeMessage extends AbstractOccupancyOcTreeNodeMessage<NormalOcTreeNodeMessage, NormalOcTreeNode>
{
   public Vector3f normal = null;
   public int regionId = PlanarRegion.NO_REGION_ID;
}
