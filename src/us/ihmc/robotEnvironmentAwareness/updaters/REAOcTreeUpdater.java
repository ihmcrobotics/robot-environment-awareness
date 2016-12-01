package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.boundingBox.OcTreeSimpleBoundingBox;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotics.geometry.transformables.Pose;

public class REAOcTreeUpdater
{
   private final NormalOcTree referenceOctree;
   private final REAOcTreeBuffer reaOcTreeBuffer;

   private final AtomicReference<Pose> latestLidarPoseReference = new AtomicReference<>(null);

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> enableNormalEstimation;
   private final AtomicReference<Boolean> clearNormals;

   private final AtomicReference<Double> minRange;
   private final AtomicReference<Double> maxRange;

   private final AtomicReference<NormalEstimationParameters> normalEstimationParameters;

   private final AtomicReference<Boolean> useBoundingBox;
   private final AtomicReference<OcTreeSimpleBoundingBox> atomicBoundingBox;

   public REAOcTreeUpdater(NormalOcTree octree, REAOcTreeBuffer buffer, REAMessager reaMessager, PacketCommunicator packetCommunicator)
   {
      this.referenceOctree = octree;
      reaOcTreeBuffer = buffer;
      referenceOctree.enableParallelComputationForNormals(true);
      referenceOctree.enableParallelInsertionOfMisses(true);

      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      enableNormalEstimation = reaMessager.createInput(REAModuleAPI.OcTreeNormalEstimationEnable, false);
      clearNormals = reaMessager.createInput(REAModuleAPI.OcTreeNormalEstimationClear, false);
      minRange = reaMessager.createInput(REAModuleAPI.OcTreeLIDARMinRange);
      maxRange = reaMessager.createInput(REAModuleAPI.OcTreeLIDARMaxRange);
      useBoundingBox = reaMessager.createInput(REAModuleAPI.OcTreeBoundingBoxEnable, true);
      atomicBoundingBox = reaMessager.createInput(REAModuleAPI.OcTreeBoundingBoxParameters);
      normalEstimationParameters = reaMessager.createInput(REAModuleAPI.OcTreeNormalEstimationParameters);

      packetCommunicator.attachListener(LidarScanMessage.class, this::handlePacket);

      referenceOctree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());
   }

   public void update()
   {
      if (!enable.get())
         return;

      handleBoundingBox();

      if (minRange.get() != null && maxRange.get() != null)
      {
         referenceOctree.setBoundsInsertRange(minRange.get(), maxRange.get());
      }

      if (normalEstimationParameters.get() != null)
         referenceOctree.setNormalEstimationParameters(normalEstimationParameters.getAndSet(null));

      if (latestLidarPoseReference.get() == null)
         return;

      Point3d sensorOrigin = latestLidarPoseReference.get().getPoint();

      boolean isBufferFull = reaOcTreeBuffer.isBufferFull();
      if (isBufferFull)
         reaOcTreeBuffer.submitBufferRequest();

      NormalOcTree bufferOctree = reaOcTreeBuffer.pollNewBuffer();

      if (bufferOctree != null)
         referenceOctree.insertNormalOcTree(sensorOrigin, bufferOctree);

      if (clearNormals.get())
      {
         referenceOctree.clearNormals();
         return;
      }

      if (bufferOctree == null || !enableNormalEstimation.get())
         return;

      referenceOctree.updateNormals();
   }

   public void clearOcTree()
   {
      referenceOctree.clear();
   }

   private void handleBoundingBox()
   {
      if (useBoundingBox.get() && atomicBoundingBox.get() != null && latestLidarPoseReference.get() != null)
      {
         OcTreeBoundingBoxWithCenterAndYaw newBoundingBox = new OcTreeBoundingBoxWithCenterAndYaw();
         newBoundingBox.setLocalBoundingBox(atomicBoundingBox.get());
         Pose lidarPose = latestLidarPoseReference.get();
         newBoundingBox.setOffset(lidarPose.getPoint());
         newBoundingBox.setYawFromQuaternion(lidarPose.getOrientation());
         newBoundingBox.update(referenceOctree.getResolution(), referenceOctree.getTreeDepth());
         referenceOctree.setBoundingBox(newBoundingBox);
      }
      else
      {
         referenceOctree.disableBoundingBox();
      }
   }

   private void handlePacket(LidarScanMessage lidarScanMessage)
   {
      latestLidarPoseReference.set(new Pose(lidarScanMessage.lidarPosition, lidarScanMessage.lidarOrientation));
   }
}
