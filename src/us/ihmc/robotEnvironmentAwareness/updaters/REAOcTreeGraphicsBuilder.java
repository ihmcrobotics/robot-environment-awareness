package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;

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

      if (octree.getRoot() != null)
      {
         NormalOcTreeMessage normalOcTreeMessage = OcTreeMessageConverter.convertToMessage(octree, regionFeaturesProvider.getOcTreePlanarRegions());
         reaMessager.submitMessage(new REAMessage(REAModuleAPI.Octree, normalOcTreeMessage));
      }

      if (Thread.interrupted())
         return;
   }

   private boolean isEnabled()
   {
      return enable.get() == null ? false : enable.get();
   }

}
