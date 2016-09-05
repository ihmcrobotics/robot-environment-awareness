package us.ihmc.robotEnvironmentAwareness.ui.ocTree;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;

import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.octoMap.ocTree.NormalOcTree;
import us.ihmc.octoMap.pointCloud.SweepCollection;
import us.ihmc.robotEnvironmentAwareness.communication.LidarPosePacket;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.tools.io.printing.PrintTools;

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
   private final AtomicReference<BoundingBox3d> atomicBoundingBox = new AtomicReference<>(new BoundingBox3d(boundingBoxMin, boundingBoxMax));

   public OcTreeUpdater(NormalOcTree octree, boolean enableInitialValue)
   {
      this.octree = octree;
      enable = new AtomicBoolean(enableInitialValue);
   }

   public void update()
   {
      if (clear.get())
      {
         octree.clear();
         clear.set(false);
         return;
      }

      if (!enable.get())
         return;

      updateSweep();

      SweepCollection newScan = newFullScanReference.getAndSet(null);

      if (newScan == null)
         return;

      octree.useBoundingBoxLimit(useBoundingBox.get());
      octree.setBoundingBoxMin(boundingBoxMin);
      octree.setBoundingBoxMax(boundingBoxMax);

      octree.ensureCapacityUnusedPools(2000000);
      octree.insertSweepCollection(newScan, minRange.get(), maxRange.get());

      if (clear.get())
      {
         octree.clear();
         clear.set(false);
         return;
      }

      long startTime = System.nanoTime();
      octree.updateSweepCollectionHitLocations(newScan, 0.1, false);
      long endTime = System.nanoTime();
      System.out.println("Exiting  updateSweepCollectionHitLocations took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));

      if (clear.get())
      {
         octree.clear();
         clear.set(false);
         return;
      }

      startTime = System.nanoTime();
      octree.updateNormals();
      endTime = System.nanoTime();
      System.out.println("Exiting  updateNormals took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));

      if (clear.get())
      {
         octree.clear();
         clear.set(false);
         return;
      }

      startTime = System.nanoTime();
      octree.updatePlanarRegionSegmentation(depthUsedForDisplay.get());
      endTime = System.nanoTime();
      System.out.println("Exiting  updatePlanarRegionSegmentation took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));

      if (clear.get())
      {
         octree.clear();
         clear.set(false);
         return;
      }
   }

   private static final boolean DEBUG = false;
   private static final boolean WAIT_FOR_FULL_SCAN = false;

   private int currentSweepId = Integer.MAX_VALUE;
   private SweepCollection sweepCollection = new SweepCollection();


   private void updateSweep()
   {
      LidarPosePacket lidarPosePacket = latestLidarPoseReference.get();
      PointCloudWorldPacket pointCloudPacket = latestPointCloudWorldPacket.getAndSet(null);
      
      if (!enable.get() || lidarPosePacket == null || pointCloudPacket == null)
         return;

      long startTime = System.nanoTime();

      int newSweepId = (int) (lidarPosePacket.getLidarJointAngle() / Math.PI);
      if (DEBUG)
         System.out.println("New sweep id = " + newSweepId + ", lidar joint angle = " + lidarPosePacket.getLidarJointAngle());

      if (!WAIT_FOR_FULL_SCAN || (newSweepId != currentSweepId && sweepCollection != null))
      {
         sweepCollection = new SweepCollection();
         newFullScanReference.set(sweepCollection);
         currentSweepId = newSweepId;
      }
      sweepCollection.setSubSampleSize(NUMBER_OF_SAMPLES);
      sweepCollection.addSweep(pointCloudPacket.decayingWorldScan, lidarPosePacket.position);

      long endTime = System.nanoTime();
      if (DEBUG)
         PrintTools.info(PacketConsumer.class, "Took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));      
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

   public NormalOcTree getOctree()
   {
      return octree;
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
