package us.ihmc.robotEnvironmentAwareness.ui.ocTree;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.octoMap.ocTree.NormalOcTree;
import us.ihmc.octoMap.pointCloud.SweepCollection;
import us.ihmc.robotEnvironmentAwareness.communication.LidarPosePacket;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.tools.io.printing.PrintTools;

public class OcTreeUpdater
{
   private static final int NUMBER_OF_SAMPLES = 50000;
   private final NormalOcTree octree;

   private final AtomicReference<LidarPosePacket> latestLidarPoseReference = new AtomicReference<>(null);
   private final AtomicReference<PointCloudWorldPacket> latestPointCloudWorldPacket = new AtomicReference<>(null);
   private final AtomicReference<SweepCollection> newFullScanReference = new AtomicReference<>(null);

   private final AtomicBoolean enable;
   private final AtomicBoolean clear = new AtomicBoolean(false);

   private final AtomicDouble minRange = new AtomicDouble(0.10);
   private final AtomicDouble maxRange = new AtomicDouble(15.0);

   public OcTreeUpdater(NormalOcTree octree, boolean enableInitialValue)
   {
      this.octree = octree;
      enable = new AtomicBoolean(enableInitialValue);
   }

   public void update()
   {
      if (clear.getAndSet(false))
      {
         octree.clear();
         return;
      }

      if (!enable.get())
         return;

      updateSweep();

      SweepCollection newScan = newFullScanReference.getAndSet(null);

      if (newScan == null)
         return;

      octree.insertSweepCollection(newScan, minRange.get(), maxRange.get());
      octree.updateNormals();
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
         sweepCollection.setSubSampleSize(NUMBER_OF_SAMPLES);
         currentSweepId = newSweepId;
      }
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

}
