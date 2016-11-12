package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LidarPosePacket;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.boundingBox.OcTreeSimpleBoundingBox;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;

public class REAOcTreeUpdater
{
   private final NormalOcTree referenceOctree;
   private final REAOcTreeBuffer reaOcTreeBuffer;

   private final AtomicReference<LidarPosePacket> latestLidarPoseReference = new AtomicReference<>(null);

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> enableNormalEstimation;
   private final AtomicReference<Boolean> clearNormals;

   private final AtomicReference<Double> minRange;
   private final AtomicReference<Double> maxRange;

   private final AtomicReference<NormalEstimationParameters> normalEstimationParameters;

   private final AtomicReference<Boolean> useBoundingBox;
   private final AtomicReference<OcTreeSimpleBoundingBox> atomicBoundingBox;

   public REAOcTreeUpdater(NormalOcTree octree, REAMessageManager inputManager, REAMessager outputMessager)
   {
      this.referenceOctree = octree;

      reaOcTreeBuffer = new REAOcTreeBuffer(inputManager, outputMessager, octree.getResolution());

      enable = inputManager.createInput(REAModuleAPI.OcTreeEnable);
      enableNormalEstimation = inputManager.createInput(REAModuleAPI.OcTreeNormalEstimationEnable);
      clearNormals = inputManager.createInput(REAModuleAPI.OcTreeNormalEstimationClear);
      minRange = inputManager.createInput(REAModuleAPI.OcTreeLIDARMinRange);
      maxRange = inputManager.createInput(REAModuleAPI.OcTreeLIDARMaxRange);
      useBoundingBox = inputManager.createInput(REAModuleAPI.OcTreeBoundingBoxEnable);
      atomicBoundingBox = inputManager.createInput(REAModuleAPI.OcTreeBoundingBoxParameters);
      normalEstimationParameters = inputManager.createInput(REAModuleAPI.OcTreeNormalEstimationParameters);
   }

   public Runnable createBufferThread()
   {
      return reaOcTreeBuffer.createBufferThread();
   }

   public boolean update(boolean performCompleteUpdate)
   {
      if (!isEnabled())
         return false;

      handleBoundingBox();

      if (minRange.get() != null && maxRange.get() != null)
      {
         referenceOctree.setBoundsInsertRange(minRange.get(), maxRange.get());
      }

      if (normalEstimationParameters.get() != null)
         referenceOctree.setNormalEstimationParameters(normalEstimationParameters.getAndSet(null));

      if (latestLidarPoseReference.get() == null)
         return false;

      Point3d sensorOrigin = latestLidarPoseReference.get().getPosition();

      boolean isBufferFull = reaOcTreeBuffer.isBufferFull();
      if (isBufferFull)
         reaOcTreeBuffer.submitBufferRequest();

      NormalOcTree bufferOctree = reaOcTreeBuffer.pollNewBuffer();

      if (bufferOctree != null)
         referenceOctree.insertNormalOcTree(sensorOrigin, bufferOctree);

      if (shouldClearNormals())
      {
         referenceOctree.clearNormals();
         return false;
      }

      if (bufferOctree == null || !isNormalEstimationEnabled())
         return false;

      if (performCompleteUpdate)
         referenceOctree.updateNormals();

      return true;
   }

   public void clearOcTree()
   {
      referenceOctree.clear();
   }

   private void handleBoundingBox()
   {
      if (isUsingBoundingBox() && atomicBoundingBox.get() != null && latestLidarPoseReference.get() != null)
      {
         OcTreeBoundingBoxWithCenterAndYaw newBoundingBox = new OcTreeBoundingBoxWithCenterAndYaw();
         newBoundingBox.setLocalBoundingBox(atomicBoundingBox.get());
         LidarPosePacket lidarPosePacket = latestLidarPoseReference.get();
         newBoundingBox.setOffset(lidarPosePacket.getPosition());
         newBoundingBox.setYawFromQuaternion(lidarPosePacket.getOrientation());
         newBoundingBox.update(referenceOctree.getResolution(), referenceOctree.getTreeDepth());
         referenceOctree.setBoundingBox(newBoundingBox);
      }
      else
      {
         referenceOctree.disableBoundingBox();
      }
   }

   public void attachListeners(PacketCommunicator packetCommunicator)
   {
      packetCommunicator.attachListener(LidarPosePacket.class, this::handlePacket);
      reaOcTreeBuffer.attachListeners(packetCommunicator);
   }

   private void handlePacket(LidarPosePacket packet)
   {
      if (packet != null)
         latestLidarPoseReference.set(new LidarPosePacket(packet));
   }

   private boolean isEnabled()
   {
      return enable.get() == null ? false : enable.get();
   }

   private boolean isNormalEstimationEnabled()
   {
      return enableNormalEstimation.get() == null ? false : enableNormalEstimation.get();
   }

   private boolean shouldClearNormals()
   {
      return clearNormals.get() == null ? false : clearNormals.getAndSet(null);
   }

   private boolean isUsingBoundingBox()
   {
      return useBoundingBox.get() == null ? false : useBoundingBox.get();
   }
}
