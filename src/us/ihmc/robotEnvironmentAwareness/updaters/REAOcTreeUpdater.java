package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.apache.commons.math3.util.Precision;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LidarScanMessage;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.boundingBox.OcTreeSimpleBoundingBox;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.ocTree.NormalOcTree.RayMissProbabilityUpdater;
import us.ihmc.jOctoMap.occupancy.OccupancyParameters;
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

   public REAOcTreeUpdater(NormalOcTree octree, REAMessageManager inputManager, REAMessager outputMessager)
   {
      this.referenceOctree = octree;
      referenceOctree.enableParallelComputationForNormals(true);
      referenceOctree.enableParallelInsertionOfMisses(true);

      reaOcTreeBuffer = new REAOcTreeBuffer(inputManager, outputMessager, octree.getResolution());

      enable = inputManager.createInput(REAModuleAPI.OcTreeEnable);
      enableNormalEstimation = inputManager.createInput(REAModuleAPI.OcTreeNormalEstimationEnable);
      clearNormals = inputManager.createInput(REAModuleAPI.OcTreeNormalEstimationClear);
      minRange = inputManager.createInput(REAModuleAPI.OcTreeLIDARMinRange);
      maxRange = inputManager.createInput(REAModuleAPI.OcTreeLIDARMaxRange);
      useBoundingBox = inputManager.createInput(REAModuleAPI.OcTreeBoundingBoxEnable);
      atomicBoundingBox = inputManager.createInput(REAModuleAPI.OcTreeBoundingBoxParameters);
      normalEstimationParameters = inputManager.createInput(REAModuleAPI.OcTreeNormalEstimationParameters);

      RayMissProbabilityUpdater rayMissProbabilityUpdater = new RayMissProbabilityUpdater()
      {
         @Override
         public double computeRayMissProbability(Point3d rayOrigin, Point3d rayEnd, Vector3d rayDirection, NormalOcTreeNode node,
               OccupancyParameters parameters)
         {
            Point3d hitLocation = new Point3d();
            node.getHitLocation(hitLocation);

            if (hitLocation.distanceSquared(rayEnd) < 0.06 * 0.06)
            {
               return 0.47;
            }
            else if (node.getNormalConsensusSize() > 10 && node.isNormalSet())
            {
               Point3d nodeHitLocation = new Point3d();
               Vector3d nodeNormal = new Vector3d();
               node.getHitLocation(nodeHitLocation);
               node.getNormal(nodeNormal);

               if (Precision.equals(Math.abs(nodeNormal.angle(rayDirection)) - Math.PI / 2.0, 0.0, Math.toRadians(30.0)))// && distanceFromPointToLine(nodeHitLocation, rayOrigin, rayEnd) > 0.01)
                  return 0.45;
               else
                  return parameters.getMissProbability();
            }
            else
            {
               return parameters.getMissProbability();
            }
         }
      };
      referenceOctree.setCustomRayMissProbabilityUpdater(rayMissProbabilityUpdater);
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

      Point3d sensorOrigin = latestLidarPoseReference.get().getPoint();

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

   public void clearBuffer()
   {
      reaOcTreeBuffer.clearBuffer();
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

   public void attachListeners(PacketCommunicator packetCommunicator)
   {
      packetCommunicator.attachListener(LidarScanMessage.class, this::handlePacket);
      reaOcTreeBuffer.attachPacketCommunicator(packetCommunicator);
   }

   private void handlePacket(LidarScanMessage lidarScanMessage)
   {
      if (lidarScanMessage != null)
         latestLidarPoseReference.set(new Pose(lidarScanMessage.lidarPosition, lidarScanMessage.lidarOrientation));
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
