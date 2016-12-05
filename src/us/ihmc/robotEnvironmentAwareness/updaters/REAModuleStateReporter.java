package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.BoundingBoxMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.converters.REAPlanarRegionsConverter;

public class REAModuleStateReporter
{
   private final REAMessager reaMessager;
   private final AtomicReference<Boolean> isLidarScanRequested;
   private final AtomicReference<Boolean> isBufferOcTreeRequested;
   private final AtomicReference<Boolean> isOcTreeRequested;
   private final AtomicReference<Boolean> isOcTreeBoundingBoxRequested;
   private final AtomicReference<Boolean> arePlanarRegionsRequested;
   private final AtomicReference<Boolean> arePlanarRegionsNodeKeysRequested;
   private final AtomicReference<Boolean> arePlanarRegionsIntersectionsRequested;

   public REAModuleStateReporter(REAMessager reaMessager, PacketCommunicator packetCommunicator)
   {
      this.reaMessager = reaMessager;
      isLidarScanRequested = reaMessager.createInput(REAModuleAPI.RequestLidarScan, false);
      isBufferOcTreeRequested = reaMessager.createInput(REAModuleAPI.RequestBuffer, false);
      isOcTreeRequested = reaMessager.createInput(REAModuleAPI.RequestOctree, false);
      isOcTreeBoundingBoxRequested = reaMessager.createInput(REAModuleAPI.RequestBoundingBox, false);
      arePlanarRegionsRequested = reaMessager.createInput(REAModuleAPI.RequestPlanarRegions, false);
      arePlanarRegionsNodeKeysRequested = reaMessager.createInput(REAModuleAPI.RequestPlanarRegionsNodeKeys, false);
      arePlanarRegionsIntersectionsRequested = reaMessager.createInput(REAModuleAPI.RequestPlanarRegionsIntersections, false);

      packetCommunicator.attachListener(LidarScanMessage.class, this::handleLidarScanMessage);
   }

   public void reportBufferOcTreeState(NormalOcTree bufferOcTree)
   {
      if (isBufferOcTreeRequested.getAndSet(false))
         reaMessager.submitMessage(REAModuleAPI.OcTreeBufferState, OcTreeMessageConverter.convertToMessage(bufferOcTree));
   }

   public void reportOcTreeState(NormalOcTree ocTree)
   {
      if (isOcTreeRequested.getAndSet(false))
         reaMessager.submitMessage(REAModuleAPI.OcTreeState, OcTreeMessageConverter.convertToMessage(ocTree));
      if (isOcTreeBoundingBoxRequested.get())
         reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxState, BoundingBoxMessageConverter.convertToMessage(ocTree.getBoundingBox()));
   }

   public void reportPlanarRegionsState(RegionFeaturesProvider regionFeaturesProvider)
   {
      if (arePlanarRegionsRequested.getAndSet(false))
         reaMessager.submitMessage(REAModuleAPI.PlanarRegionsState, REAPlanarRegionsConverter.createPlanarRegionsListMessage(regionFeaturesProvider));
      if (arePlanarRegionsNodeKeysRequested.getAndSet(false))
         reaMessager.submitMessage(REAModuleAPI.PlanarRegionsNodeState, REAPlanarRegionsConverter.createPlanarRegionNodeKeysMessages(regionFeaturesProvider));
      if (arePlanarRegionsIntersectionsRequested.getAndSet(false))
         reaMessager.submitMessage(REAModuleAPI.PlanarRegionsIntersectionState, REAPlanarRegionsConverter.createLineSegment3dMessages(regionFeaturesProvider));
   }

   private void handleLidarScanMessage(LidarScanMessage message)
   {
      if (isLidarScanRequested.getAndSet(false))
         reaMessager.submitMessage(REAModuleAPI.LidarScanState, message);
   }
}
