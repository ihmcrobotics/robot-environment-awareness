package us.ihmc.robotEnvironmentAwareness.updaters;

import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class REAOcTreeGraphicsBuilder
{

   private final NormalOcTree octree;
   private final RegionFeaturesProvider regionFeaturesProvider;

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Integer> treeDepthForDisplay;

   private final AtomicReference<Boolean> useOcTreeBoundingBox;

   private final AtomicBoolean processPropertyChange = new AtomicBoolean(false);

   private final REAMessager reaMessager;

   public REAOcTreeGraphicsBuilder(NormalOcTree octree, RegionFeaturesProvider regionFeaturesProvider, REAMessager reaMessager)
   {
      this.octree = octree;
      this.regionFeaturesProvider = regionFeaturesProvider;
      this.reaMessager = reaMessager;
      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable);
      treeDepthForDisplay = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsDepth);
      useOcTreeBoundingBox = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsBoundingBoxEnable);
   }

   public void update()
   {

      if (!isEnabled() && !processPropertyChange.get())
         return;

      processPropertyChange.set(false);

//      int currentDepth = getCurrentTreeDepthForDisplay();
//
//      //TODO  get all nodes from the octree
//      Set<NormalOcTreeNode> nodes = getNodes(octree, currentDepth);
//
//      ArrayList<OctreeNodeData> nodesData = new ArrayList<>(octree.size());
//
//      //TODO Go through all nodes for all regions
//      for (OcTreeNodePlanarRegion ocTreeNodePlanarRegion : regionFeaturesProvider.getOcTreePlanarRegions())
//      {
//         int regionId = ocTreeNodePlanarRegion.getId();
//         for (NormalOcTreeNode node : ocTreeNodePlanarRegion)
//         {
//            nodes.remove(node);
//            nodesData.add(new OctreeNodeData(node, regionId));
//         }
//      }
//
//      // only go through nodes that don't belong to any region
//      for (NormalOcTreeNode node : nodes)
//      {
//         nodesData.add(new OctreeNodeData(node, OcTreeNodePlanarRegion.NO_REGION_ID));
//      }
//
//      float size = (float) OcTreeKeyConversionTools.computeNodeSize(octree.getTreeDepth(), octree.getResolution(), octree.getTreeDepth());

      if(octree.getRoot() != null)
      {
         NormalOcTreeMessage normalOcTreeMessage = OcTreeMessageConverter.convertToMessage(octree, regionFeaturesProvider.getOcTreePlanarRegions());
         normalOcTreeMessage.messageID = REAModuleAPI.OctreeMessageID;
         reaMessager.getPacketCommunicator().send(normalOcTreeMessage);
      }

      if (Thread.interrupted())
         return;
   }

//   private Set<NormalOcTreeNode> getNodes(NormalOcTree octree, int currentDepth)
//   {
//      OcTreeIterable<NormalOcTreeNode> iterable = OcTreeIteratorFactory.createLeafIteratable(octree.getRoot(), currentDepth);
//      if (useOcTreeBoundingBox())
//         iterable.setRule(OcTreeIteratorFactory.leavesInsideBoundingBoxOnly(octree.getBoundingBox()));
//
//      Set<NormalOcTreeNode> nodes = new HashSet<>();
//      iterable.forEach(nodes::add);
//      return nodes;
//   }

   private boolean isEnabled()
   {
      return enable.get() == null ? false : enable.get();
   }

   private int getCurrentTreeDepthForDisplay()
   {
      return treeDepthForDisplay.get() == null ? octree.getTreeDepth() : treeDepthForDisplay.get();
   }

   private boolean useOcTreeBoundingBox()
   {
      return useOcTreeBoundingBox.get() == null ? false : useOcTreeBoundingBox.get();
   }

}
