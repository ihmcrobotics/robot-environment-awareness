package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.converters.REAPlanarRegionsConverter;

public class REAModuleStateReporter
{
   private final REAMessager reaMessager;
   private final AtomicReference<Boolean> isLidarScanRequested;
   private final AtomicReference<Boolean> isBufferOcTreeRequested;
   private final AtomicReference<Boolean> isOcTreeRequested;
   private final AtomicReference<Boolean> arePlanarRegionsRequested;

   public REAModuleStateReporter(REAMessager reaMessager, PacketCommunicator packetCommunicator)
   {
      this.reaMessager = reaMessager;
      isLidarScanRequested = reaMessager.createInput(REAModuleAPI.RequestLidarScan, false);
      isBufferOcTreeRequested = reaMessager.createInput(REAModuleAPI.RequestBuffer, false);
      isOcTreeRequested = reaMessager.createInput(REAModuleAPI.RequestOctree, false);
      arePlanarRegionsRequested = reaMessager.createInput(REAModuleAPI.RequestPlanarRegions, false);

      packetCommunicator.attachListener(LidarScanMessage.class, this::handleLidarScanMessage);
   }

   public void reportBufferOcTreeState(NormalOcTree bufferOcTree)
   {
      if (isBufferOcTreeRequested.getAndSet(false))
         reaMessager.submitMessage(REAModuleAPI.BufferState, OcTreeMessageConverter.convertToMessage(bufferOcTree));
   }

   public void reportOcTreeState(NormalOcTree ocTree)
   {
      if (isOcTreeRequested.getAndSet(false))
         reaMessager.submitMessage(REAModuleAPI.OcTreeState, OcTreeMessageConverter.convertToMessage(ocTree));
   }

   public void reportPlanarRegionsState(RegionFeaturesProvider regionFeaturesProvider)
   {
      if (arePlanarRegionsRequested.getAndSet(false))
         reaMessager.submitMessage(REAModuleAPI.PlanarRegionsState, REAPlanarRegionsConverter.createPlanarRegionsListMessage(regionFeaturesProvider));
   }

   private void handleLidarScanMessage(LidarScanMessage message)
   {
      if (isLidarScanRequested.getAndSet(false))
         reaMessager.submitMessage(REAModuleAPI.LidarScanState, message);
   }
}
