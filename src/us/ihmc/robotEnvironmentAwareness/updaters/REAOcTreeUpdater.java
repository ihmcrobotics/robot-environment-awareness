package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LidarPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.boundingBox.OcTreeSimpleBoundingBox;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;

public class REAOcTreeUpdater
{
   private static final int NUMBER_OF_SAMPLES = 100000;

   private final NormalOcTree referenceOctree;
   private final NormalOcTree bufferOctree;

   private final AtomicReference<LidarPosePacket> latestLidarPoseReference = new AtomicReference<>(null);
   private final AtomicReference<PointCloudWorldPacket> latestPointCloudWorldPacket = new AtomicReference<>(null);
   private final AtomicReference<ScanCollection> newFullScanReference = new AtomicReference<>(null);

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> enableNormalEstimation;
   private final AtomicReference<Boolean> clearNormals;
   private final AtomicReference<Double> bufferSize;

   private final AtomicReference<Double> minRange;
   private final AtomicReference<Double> maxRange;

   private final AtomicReference<NormalEstimationParameters> normalEstimationParameters;

   private final AtomicReference<Boolean> useBoundingBox;
   private final AtomicReference<OcTreeSimpleBoundingBox> atomicBoundingBox;

   public REAOcTreeUpdater(NormalOcTree octree, REAMessageManager inputManager, REAMessager outputMessager)
   {
      this.referenceOctree = octree;
      bufferOctree = new NormalOcTree(octree.getResolution());

      enable = inputManager.createInput(REAModuleAPI.OcTreeEnable);
      enableNormalEstimation = inputManager.createInput(REAModuleAPI.OcTreeNormalEstimationEnable);
      clearNormals = inputManager.createInput(REAModuleAPI.OcTreeNormalEstimationClear);
      minRange = inputManager.createInput(REAModuleAPI.OcTreeLIDARMinRange);
      maxRange = inputManager.createInput(REAModuleAPI.OcTreeLIDARMaxRange);
      useBoundingBox = inputManager.createInput(REAModuleAPI.OcTreeBoundingBoxEnable);
      atomicBoundingBox = inputManager.createInput(REAModuleAPI.OcTreeBoundingBoxParameters);
      normalEstimationParameters = inputManager.createInput(REAModuleAPI.OcTreeNormalEstimationParameters);
      bufferSize = inputManager.createInput(REAModuleAPI.OcTreeBufferSize, 10000.0);
   }

   public boolean update(boolean performCompleteUpdate)
   {
      if (!isEnabled())
         return false;

      updateScanCollection();

      ScanCollection newScan = newFullScanReference.getAndSet(null);

      if (newScan == null)
         return false;

      handleBoundingBox();

      if (minRange.get() != null && maxRange.get() != null)
      {
         bufferOctree.setBoundsInsertRange(minRange.get(), maxRange.get());
         referenceOctree.setBoundsInsertRange(minRange.get(), maxRange.get());
      }

      if (normalEstimationParameters.get() != null)
         referenceOctree.setNormalEstimationParameters(normalEstimationParameters.getAndSet(null));

      Point3d sensorOrigin = newScan.getScan(newScan.getNumberOfScans() - 1).getSensorOrigin();
      bufferOctree.insertScanCollection(newScan);

      int numberOfLeafNodesInBuffer = bufferOctree.getNumberOfLeafNodes();
      boolean updateReference = numberOfLeafNodesInBuffer >= bufferSize.get().intValue();

      if (updateReference)
      {
         referenceOctree.insertNormalOcTree(sensorOrigin, bufferOctree);
         bufferOctree.clear();
      }

      if (shouldClearNormals())
      {
         referenceOctree.clearNormals();
         return false;
      }

      if (!updateReference || !isNormalEstimationEnabled())
         return false;

      if (performCompleteUpdate)
         referenceOctree.updateNormals();

      return true;
   }

   public void clearOcTree()
   {
      referenceOctree.clear();
      bufferOctree.clear();
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
         bufferOctree.setBoundingBox(newBoundingBox);
      }
      else
      {
         referenceOctree.disableBoundingBox();
         bufferOctree.disableBoundingBox();
      }
   }

   private void updateScanCollection()
   {
      LidarPosePacket lidarPosePacket = latestLidarPoseReference.get();
      PointCloudWorldPacket pointCloudPacket = latestPointCloudWorldPacket.getAndSet(null);

      if (!isEnabled() || lidarPosePacket == null || pointCloudPacket == null)
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

   public NormalOcTree getBufferOctree()
   {
      return bufferOctree;
   }
}
