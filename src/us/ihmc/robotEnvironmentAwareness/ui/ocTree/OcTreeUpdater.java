package us.ihmc.robotEnvironmentAwareness.ui.ocTree;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;

import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.octoMap.ocTree.implementations.NormalOcTree;
import us.ihmc.octoMap.pointCloud.SweepCollection;
import us.ihmc.robotEnvironmentAwareness.communication.LidarPosePacket;

public class OcTreeUpdater
{
   private static final int NUMBER_OF_SAMPLES = 100000;
   private final NormalOcTree octree;

   private final AtomicReference<LidarPosePacket> latestLidarPoseReference = new AtomicReference<>(null);
   private final AtomicReference<PointCloudWorldPacket> latestPointCloudWorldPacket = new AtomicReference<>(null);
   private final AtomicReference<SweepCollection> newFullScanReference = new AtomicReference<>(null);

   private final AtomicBoolean enable;
   private final AtomicBoolean clear = new AtomicBoolean(false);

   private final AtomicDouble minRange = new AtomicDouble(0.20);
   private final AtomicDouble maxRange = new AtomicDouble(3.0);

   private final AtomicInteger depthUsedForDisplay = new AtomicInteger(-1);

   private final AtomicBoolean useBoundingBox = new AtomicBoolean(false);
   private final Point3d boundingBoxMin = new Point3d(-0.0, -2.0, -1.0);
   private final Point3d boundingBoxMax = new Point3d(10.0, 2.0, 1.0);
   private final AtomicReference<OcTreeBoundingBoxWithCenterAndYaw> atomicBoundingBox;

   public OcTreeUpdater(NormalOcTree octree, boolean enableInitialValue)
   {
      this.octree = octree;
      enable = new AtomicBoolean(enableInitialValue);
      double resolution = octree.getResolution();
      int treeDepth = octree.getTreeDepth();
      atomicBoundingBox = new AtomicReference<>(new OcTreeBoundingBoxWithCenterAndYaw(boundingBoxMin, boundingBoxMax, resolution, treeDepth));
   }

   public void update()
   {
      if (clear.get())
      {
         octree.clear();
         clear.set(false);
         return;
      }

      if (!enable.get() || Thread.interrupted())
         return;

      updateSweep();

      SweepCollection newScan = newFullScanReference.getAndSet(null);

      if (newScan == null)
         return;

      handleBoundingBox();

      octree.setBoundsInsertRange(minRange.get(), maxRange.get());
      octree.insertSweepCollection(newScan);

      if (Thread.interrupted())
         return;

      if (clear.get())
      {
         octree.clear();
         clear.set(false);
         return;
      }

      octree.updateNormalsAndPlanarRegions(depthUsedForDisplay.get());

      if (Thread.interrupted())
         return;

      if (clear.get())
      {
         octree.clear();
         clear.set(false);
         return;
      }
   }

   private void handleBoundingBox()
   {
      if (useBoundingBox.get())
      {
         OcTreeBoundingBoxWithCenterAndYaw newBoundingBox = atomicBoundingBox.get();
         LidarPosePacket lidarPosePacket = latestLidarPoseReference.get();
         newBoundingBox.setOffsetCoordinate(lidarPosePacket.getPosition());
         newBoundingBox.setYawFromQuaternion(lidarPosePacket.getOrientation());
         newBoundingBox.update(octree.getResolution(), octree.getTreeDepth());
         octree.setBoundingBox(newBoundingBox);
      }
      else
         octree.disableBoundingBox();
   }

   private void updateSweep()
   {
      LidarPosePacket lidarPosePacket = latestLidarPoseReference.get();
      PointCloudWorldPacket pointCloudPacket = latestPointCloudWorldPacket.getAndSet(null);
      
      if (!enable.get() || lidarPosePacket == null || pointCloudPacket == null)
         return;

      SweepCollection sweepCollection = new SweepCollection();
      newFullScanReference.set(sweepCollection);
      sweepCollection.setSubSampleSize(NUMBER_OF_SAMPLES);
      sweepCollection.addSweep(pointCloudPacket.decayingWorldScan, lidarPosePacket.position);
   }

   public void setEnable(boolean enable)
   {
      this.enable.set(enable);
   }

   public void clear()
   {
      clear.set(true);
   }

   public boolean hasProcessedClear()
   {
      return !clear.get();
   }

   public void setMinRange(double minRange)
   {
      this.minRange.set(minRange);
   }

   public double getMinRange()
   {
      return minRange.get();
   }

   public void setMaxRange(double maxRange)
   {
      this.maxRange.set(maxRange);
   }

   public double getMaxRange()
   {
      return maxRange.get();
   }

   public boolean isBoundingBoxEnabled()
   {
      return useBoundingBox.get();
   }

   public void enableBoundingBox(boolean enable)
   {
      useBoundingBox.set(enable);
   }

   public OcTreeBoundingBoxWithCenterAndYaw getBoundingBox()
   {
      return atomicBoundingBox.get();
   }

   public void setBoundingBox(OcTreeBoundingBoxWithCenterAndYaw boundingBox)
   {
      atomicBoundingBox.set(boundingBox);
   }

   public void attachListeners(PacketCommunicator packetCommunicator)
   {
      packetCommunicator.attachListener(LidarPosePacket.class, getLidarPosePacketConsumer());
      packetCommunicator.attachListener(PointCloudWorldPacket.class, getPointCloudWorldPacketConsumer());
   }

   private PacketConsumer<LidarPosePacket> getLidarPosePacketConsumer()
   {
      PacketConsumer<LidarPosePacket> packetConsumer = new PacketConsumer<LidarPosePacket>()
      {
         @Override
         public void receivedPacket(LidarPosePacket packet)
         {
            if (packet == null)
               return;

            latestLidarPoseReference.set(new LidarPosePacket(packet));
         }
      };
      return packetConsumer;
   }

   private PacketConsumer<PointCloudWorldPacket> getPointCloudWorldPacketConsumer()
   {
      PacketConsumer<PointCloudWorldPacket> packetConsumer = new PacketConsumer<PointCloudWorldPacket>()
      {
         @Override
         public void receivedPacket(PointCloudWorldPacket packet)
         {
            if (packet != null)
               latestPointCloudWorldPacket.set(packet);
         }
      };
      return packetConsumer;
   }

   public void setTreeDepthUsedForDisplay(int newDepth)
   {
      depthUsedForDisplay.set(newDepth);
   }
}
