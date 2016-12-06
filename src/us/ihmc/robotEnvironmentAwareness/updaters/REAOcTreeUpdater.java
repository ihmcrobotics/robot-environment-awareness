package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotics.geometry.transformables.Pose;

public class REAOcTreeUpdater
{
   private final REAMessager reaMessager;
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
   private final AtomicReference<BoundingBoxParametersMessage> atomicBoundingBoxParameters;

   public REAOcTreeUpdater(NormalOcTree octree, REAOcTreeBuffer buffer, REAMessager reaMessager, PacketCommunicator packetCommunicator)
   {
      this.referenceOctree = octree;
      reaOcTreeBuffer = buffer;
      this.reaMessager = reaMessager;
      referenceOctree.enableParallelComputationForNormals(true);
      referenceOctree.enableParallelInsertionOfMisses(true);

      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      enableNormalEstimation = reaMessager.createInput(REAModuleAPI.NormalEstimationEnable, false);
      clearNormals = reaMessager.createInput(REAModuleAPI.NormalEstimationClear, false);
      minRange = reaMessager.createInput(REAModuleAPI.LidarMinRange, 0.2);
      maxRange = reaMessager.createInput(REAModuleAPI.LidarMaxRange, 5.0);
      useBoundingBox = reaMessager.createInput(REAModuleAPI.OcTreeBoundingBoxEnable, true);
      atomicBoundingBoxParameters = reaMessager.createInput(REAModuleAPI.OcTreeBoundingBoxParameters, new BoundingBoxParametersMessage(0.0f, -2.0f, -3.0f, 5.0f, 2.0f, 0.5f));
      normalEstimationParameters = reaMessager.createInput(REAModuleAPI.NormalEstimationParameters, new NormalEstimationParameters());

      reaMessager.registerTopicListener(REAModuleAPI.RequestEntireModuleState, messageContent -> sendCurrentState());

      packetCommunicator.attachListener(LidarScanMessage.class, this::handlePacket);

      referenceOctree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());
   }

   private void sendCurrentState()
   {
      reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, enable.get());
      reaMessager.submitMessage(REAModuleAPI.NormalEstimationEnable, enableNormalEstimation.get());
      reaMessager.submitMessage(REAModuleAPI.LidarMinRange, minRange.get());
      reaMessager.submitMessage(REAModuleAPI.LidarMaxRange, maxRange.get());
      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxEnable, useBoundingBox.get());

      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxParameters, atomicBoundingBoxParameters.get());
      reaMessager.submitMessage(REAModuleAPI.NormalEstimationParameters, normalEstimationParameters.get());
   }

   public void loadConfiguration(FilePropertyHelper filePropertyHelper)
   {
      Boolean enableFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.OcTreeEnable.getName());
      if (enableFile != null)
         enable.set(enableFile);
      Boolean enableNormalEstimationFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.NormalEstimationEnable.getName());
      if (enableNormalEstimationFile != null)
         enableNormalEstimation.set(enableNormalEstimationFile);
      String normalEstimationParametersFile = filePropertyHelper.loadProperty(REAModuleAPI.NormalEstimationParameters.getName());
      if (normalEstimationParametersFile != null)
         normalEstimationParameters.set(NormalEstimationParameters.parse(normalEstimationParametersFile));
      Boolean useBoundingBoxFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.OcTreeBoundingBoxEnable.getName());
      if (useBoundingBoxFile != null)
         useBoundingBox.set(useBoundingBoxFile);
      String boundingBoxParametersFile = filePropertyHelper.loadProperty(REAModuleAPI.OcTreeBoundingBoxParameters.getName());
      if (boundingBoxParametersFile != null)
         atomicBoundingBoxParameters.set(BoundingBoxParametersMessage.parse(boundingBoxParametersFile));
      Double minRangeFile = filePropertyHelper.loadDoubleProperty(REAModuleAPI.LidarMinRange.getName());
      if (minRangeFile != null)
         minRange.set(minRangeFile);
      Double maxRangeFile = filePropertyHelper.loadDoubleProperty(REAModuleAPI.LidarMaxRange.getName());
      if (maxRangeFile != null)
         maxRange.set(maxRangeFile);
   }

   public void saveConfiguration(FilePropertyHelper filePropertyHelper)
   {
      filePropertyHelper.saveProperty(REAModuleAPI.OcTreeEnable.getName(), enable.get());
      filePropertyHelper.saveProperty(REAModuleAPI.NormalEstimationEnable.getName(), enableNormalEstimation.get());
      filePropertyHelper.saveProperty(REAModuleAPI.OcTreeBoundingBoxEnable.getName(), useBoundingBox.get());

      filePropertyHelper.saveProperty(REAModuleAPI.NormalEstimationParameters.getName(), normalEstimationParameters.get().toString());
      filePropertyHelper.saveProperty(REAModuleAPI.OcTreeBoundingBoxParameters.getName(), atomicBoundingBoxParameters.get().toString());
      filePropertyHelper.saveProperty(REAModuleAPI.LidarMinRange.getName(), minRange.get());
      filePropertyHelper.saveProperty(REAModuleAPI.LidarMaxRange.getName(), maxRange.get());
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

      referenceOctree.setNormalEstimationParameters(normalEstimationParameters.get());

      if (latestLidarPoseReference.get() == null)
         return;

      Point3d sensorOrigin = latestLidarPoseReference.get().getPoint();

      boolean isBufferFull = reaOcTreeBuffer.isBufferFull();
      if (isBufferFull)
         reaOcTreeBuffer.submitBufferRequest();

      NormalOcTree bufferOctree = reaOcTreeBuffer.pollNewBuffer();

      if (bufferOctree != null)
         referenceOctree.insertNormalOcTree(sensorOrigin, bufferOctree);

      if (clearNormals.getAndSet(false))
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
      if (!useBoundingBox.get())
      {
         referenceOctree.disableBoundingBox();
         return;
      }

      OcTreeBoundingBoxWithCenterAndYaw boundingBox = new OcTreeBoundingBoxWithCenterAndYaw();

      Point3d min = atomicBoundingBoxParameters.get().getMin();
      Point3d max = atomicBoundingBoxParameters.get().getMax();
      boundingBox.setLocalMinMaxCoordinates(min, max);

      if (latestLidarPoseReference.get() != null)
      {
         Pose lidarPose = latestLidarPoseReference.get();
         boundingBox.setOffset(lidarPose.getPoint());
         boundingBox.setYawFromQuaternion(lidarPose.getOrientation());
      }

      boundingBox.update(referenceOctree.getResolution(), referenceOctree.getTreeDepth());
      referenceOctree.setBoundingBox(boundingBox);
   }

   private void handlePacket(LidarScanMessage lidarScanMessage)
   {
      latestLidarPoseReference.set(new Pose(lidarScanMessage.lidarPosition, lidarScanMessage.lidarOrientation));
   }
}
