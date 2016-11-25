package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LidarPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;

public class REAOcTreeBuffer
{
   private static final int NUMBER_OF_SAMPLES = 100000;

   private final AtomicReference<LidarPosePacket> latestLidarPoseReference = new AtomicReference<>(null);
   private final AtomicReference<PointCloudWorldPacket> latestPointCloudWorldPacket = new AtomicReference<>(null);
   private final AtomicReference<ScanCollection> newFullScanReference = new AtomicReference<>(null);

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Double> bufferSize;

   private final AtomicBoolean clearBuffer = new AtomicBoolean(false);
   private final AtomicBoolean isBufferFull = new AtomicBoolean(false);
   private final AtomicBoolean isBufferRequested = new AtomicBoolean(false);
   private final AtomicReference<NormalOcTree> newBuffer = new AtomicReference<>(null);
   
   private final REAOcTreeBufferGraphicsBuilder graphicsBuilder;

   private final double octreeResolution;

   public REAOcTreeBuffer(double octreeResolution, REAMessager reaMessager)
   {
      this.octreeResolution = octreeResolution;
      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      bufferSize = reaMessager.createInput(REAModuleAPI.OcTreeBufferSize, 10000.0);
      graphicsBuilder = new REAOcTreeBufferGraphicsBuilder(reaMessager);
   }

   public Runnable createBufferThread()
   {
      return new Runnable()
      {
         private NormalOcTree bufferOctree = new NormalOcTree(octreeResolution);

         @Override
         public void run()
         {
            updateScanCollection();
            ScanCollection newScan = newFullScanReference.getAndSet(null);

            if (clearBuffer.getAndSet(false))
            {
               bufferOctree.clear();
               isBufferFull.set(false);
               isBufferRequested.set(false);
               return;
            }

            if (newScan == null)
               return;

            bufferOctree.insertScanCollection(newScan, false);

            int numberOfLeafNodesInBuffer = bufferOctree.getNumberOfLeafNodes();
            isBufferFull.set(numberOfLeafNodesInBuffer >= bufferSize.get().intValue());

            if (isBufferRequested.get())
            {
               newBuffer.set(bufferOctree);
               bufferOctree = new NormalOcTree(octreeResolution);
               isBufferRequested.set(false);
            }

            graphicsBuilder.update(bufferOctree, newScan);  // TODO garder le graphic builder --> send stuff at this point copy octree and send it over the network
         }
      };
   }

   public void clearBuffer()
   {
      clearBuffer.set(true);
   }

   public void submitBufferRequest()
   {
      isBufferRequested.set(true);
   }

   public boolean isBufferFull()
   {
      return isBufferFull.get();
   }

   public NormalOcTree pollNewBuffer()
   {
      return newBuffer.getAndSet(null);
   }

   private void updateScanCollection()
   {
      LidarPosePacket lidarPosePacket = latestLidarPoseReference.get();
      PointCloudWorldPacket pointCloudPacket = latestPointCloudWorldPacket.getAndSet(null);

      if (!enable.get() || lidarPosePacket == null || pointCloudPacket == null)
         return;

      ScanCollection scanCollection = new ScanCollection();
      newFullScanReference.set(scanCollection);
      scanCollection.setSubSampleSize(NUMBER_OF_SAMPLES);
      scanCollection.addScan(pointCloudPacket.decayingWorldScan, lidarPosePacket.position);
   }

   public void attachListeners(PacketCommunicator packetCommunicator)
   {
      packetCommunicator.attachListener(LidarPosePacket.class, this::handlePacket);
      packetCommunicator.attachListener(PointCloudWorldPacket.class, this::handlePacket);
   }

   private void handlePacket(LidarPosePacket packet)
   {
      if (packet != null)
         latestLidarPoseReference.set(new LidarPosePacket(packet));
   }

   private void handlePacket(PointCloudWorldPacket packet)
   {
      if (packet != null)
         latestPointCloudWorldPacket.set(packet);
   }
}
