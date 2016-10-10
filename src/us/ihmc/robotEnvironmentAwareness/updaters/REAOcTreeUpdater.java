package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.octoMap.boundingBox.OcTreeSimpleBoundingBox;
import us.ihmc.octoMap.ocTree.implementations.NormalEstimationParameters;
import us.ihmc.octoMap.ocTree.implementations.NormalOcTree;
import us.ihmc.octoMap.pointCloud.SweepCollection;
import us.ihmc.robotEnvironmentAwareness.communication.LidarPosePacket;

public class REAOcTreeUpdater
{
   private static final int NUMBER_OF_SAMPLES = 100000;
   private final NormalOcTree octree;

   private final AtomicReference<LidarPosePacket> latestLidarPoseReference = new AtomicReference<>(null);
   private final AtomicReference<PointCloudWorldPacket> latestPointCloudWorldPacket = new AtomicReference<>(null);
   private final AtomicReference<SweepCollection> newFullScanReference = new AtomicReference<>(null);

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> clear;

   private final AtomicReference<Double> minRange;
   private final AtomicReference<Double> maxRange;

   private final AtomicReference<NormalEstimationParameters> normalEstimationParameters;

   private final AtomicReference<Boolean> useBoundingBox;
   private final AtomicReference<OcTreeSimpleBoundingBox> atomicBoundingBox;

   public REAOcTreeUpdater(NormalOcTree octree, REAMessageManager inputManager, REAMessager outputMessager)
   {
      this.octree = octree;

      enable = inputManager.createInput(REAModuleAPI.OcTreeEnable);
      clear = inputManager.createInput(REAModuleAPI.OcTreeClear);
      minRange = inputManager.createInput(REAModuleAPI.OcTreeLIDARMinRange);
      maxRange = inputManager.createInput(REAModuleAPI.OcTreeLIDARMaxRange);
      useBoundingBox = inputManager.createInput(REAModuleAPI.OcTreeBoundingBoxEnable);
      atomicBoundingBox = inputManager.createInput(REAModuleAPI.OcTreeBoundingBoxParameters);
      normalEstimationParameters = inputManager.createInput(REAModuleAPI.OcTreeNormalEstimationParameters);
   }

   public void update(boolean performCompleteUpdate)
   {
      if (shouldClear())
      {
         clear();
         return;
      }

      if (!isEnabled() || Thread.interrupted())
         return;

      updateSweep();

      SweepCollection newScan = newFullScanReference.getAndSet(null);

      if (newScan == null)
      {
         if (performCompleteUpdate)
            newScan = new SweepCollection();
         else
            return;
      }

      handleBoundingBox();

      if (minRange.get() != null && maxRange.get() != null)
         octree.setBoundsInsertRange(minRange.get(), maxRange.get());

      if (normalEstimationParameters.get() != null)
         octree.setNormalEstimationParameters(normalEstimationParameters.getAndSet(null));

      octree.update(newScan, performCompleteUpdate);

      if (Thread.interrupted())
         return;

      if (shouldClear())
      {
         clear();
         return;
      }
   }

   private void clear()
   {
      octree.clear();
   }

   private void handleBoundingBox()
   {
      if (isUsingBoundingBox() && atomicBoundingBox.get() != null)
      {
         OcTreeBoundingBoxWithCenterAndYaw newBoundingBox = new OcTreeBoundingBoxWithCenterAndYaw(octree.getResolution(), octree.getTreeDepth());
         newBoundingBox.setLocalBoundingBox(atomicBoundingBox.get());
         LidarPosePacket lidarPosePacket = latestLidarPoseReference.get();
         newBoundingBox.setOffsetCoordinate(lidarPosePacket.getPosition());
         newBoundingBox.setYawFromQuaternion(lidarPosePacket.getOrientation());
         octree.setBoundingBox(newBoundingBox);
      }
      else
         octree.disableBoundingBox();
   }

   private void updateSweep()
   {
      LidarPosePacket lidarPosePacket = latestLidarPoseReference.get();
      PointCloudWorldPacket pointCloudPacket = latestPointCloudWorldPacket.getAndSet(null);

      if (!isEnabled() || lidarPosePacket == null || pointCloudPacket == null)
         return;

      SweepCollection sweepCollection = new SweepCollection();
      newFullScanReference.set(sweepCollection);
      sweepCollection.setSubSampleSize(NUMBER_OF_SAMPLES);
      sweepCollection.addSweep(pointCloudPacket.decayingWorldScan, lidarPosePacket.position);
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

   private boolean isEnabled()
   {
      return enable.get() == null ? false : enable.get();
   }

   private boolean shouldClear()
   {
      return clear.get() == null ? false : clear.getAndSet(null);
   }

   private boolean isUsingBoundingBox()
   {
      return useBoundingBox.get() == null ? false : useBoundingBox.get();
   }
}
