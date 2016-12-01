package us.ihmc.robotEnvironmentAwareness.updaters;

import java.io.IOException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.tools.JOctoMapTools;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationKryoNetClassList;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessagerOverNetwork;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessagerSharedVariables;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class LIDARBasedREAModule
{
   private static final String ocTreeTimeReport = "OcTree update took: ";
   private static final String planarRegionsTimeReport = "OcTreePlanarRegion update took: ";
   private static final String graphicsTimeReport = "OcTreeGraphics update took: ";

   private final TimeReporter timeReporter = new TimeReporter(this);

   private static final int THREAD_PERIOD_MILLISECONDS = 200;
   private static final int BUFFER_THREAD_PERIOD_MILLISECONDS = 10;
   private static final double GRAPHICS_REFRESH_PERIOD = 2.0; // in seconds
   private static final double OCTREE_RESOLUTION = 0.02;
   protected static final boolean DEBUG = true;

   private final String networkManagerHost = NetworkParameters.getHost(NetworkParameterKeys.networkManager);
   private final PacketCommunicator packetCommunicator;

   private final NormalOcTree mainOctree = new NormalOcTree(OCTREE_RESOLUTION);

   private final REAOcTreeBuffer bufferUpdater;
   private final REAOcTreeUpdater mainUpdater;
   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdater;

   private final REAOcTreeGraphicsBuilder graphicsBuilder;

   private final REAPlanarRegionNetworkProvider planarRegionNetworkProvider;

   private final AtomicReference<Boolean> clearOcTree;

   private ScheduledExecutorService executorService = Executors.newScheduledThreadPool(3, ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private ScheduledFuture<?> scheduled;
   private final REAMessager reaMessager;

   private LIDARBasedREAModule(REAMessager reaMessager)
   {
      this.reaMessager = reaMessager;
      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(networkManagerHost, NetworkPorts.REA_MODULE_PORT,
            new IHMCCommunicationKryoNetClassList());

      bufferUpdater = new REAOcTreeBuffer(mainOctree.getResolution(), reaMessager, packetCommunicator);
      mainUpdater = new REAOcTreeUpdater(mainOctree, bufferUpdater, reaMessager, packetCommunicator);

      planarRegionFeatureUpdater = new REAPlanarRegionFeatureUpdater(mainOctree, reaMessager);
      graphicsBuilder = new REAOcTreeGraphicsBuilder(mainOctree, planarRegionFeatureUpdater, reaMessager);

      planarRegionNetworkProvider = new REAPlanarRegionNetworkProvider(planarRegionFeatureUpdater, packetCommunicator);
      clearOcTree = reaMessager.createInput(REAModuleAPI.OcTreeClear, false);
   }

   private final AtomicDouble lastCompleteUpdate = new AtomicDouble(Double.NaN);
   private final AtomicDouble lastGraphicsUpdate = new AtomicDouble(Double.NaN);

   private void mainUpdate()
   {
      if (isThreadInterrupted())
         return;

      double currentTime = JOctoMapTools.nanoSecondsToSeconds(System.nanoTime());

      boolean performGraphicsUpdate = (Double.isNaN(lastGraphicsUpdate.get()) || currentTime - lastGraphicsUpdate.get() >= GRAPHICS_REFRESH_PERIOD);

      boolean ocTreeUpdateSuccess = true;

      try
      {
         if (clearOcTree.getAndSet(false))
         {
            bufferUpdater.clearBuffer();
            mainUpdater.clearOcTree();
            planarRegionFeatureUpdater.clearOcTree();
         }
         else
         {
            timeReporter.run(mainUpdater::update, ocTreeTimeReport);

            if (isThreadInterrupted())
               return;

            timeReporter.run(planarRegionFeatureUpdater::update, planarRegionsTimeReport);

            planarRegionNetworkProvider.update(ocTreeUpdateSuccess);
         }

         if (isThreadInterrupted())
            return;

         timeReporter.run(graphicsBuilder::update, graphicsTimeReport);

         if (planarRegionNetworkProvider.pollClearRequest())
            clearOcTree.set(true);
      }
      catch (Exception e)
      {
         if (DEBUG)
         {
            e.printStackTrace();
         }
         else
         {
            PrintTools.error(LIDARBasedREAModule.class, e.getClass().getSimpleName());
         }
      }

      currentTime = JOctoMapTools.nanoSecondsToSeconds(System.nanoTime());
      if (ocTreeUpdateSuccess)
         lastCompleteUpdate.set(currentTime);
      if (performGraphicsUpdate)
         lastGraphicsUpdate.set(currentTime);
   }

   private boolean isThreadInterrupted()
   {
      return Thread.interrupted() || scheduled == null || scheduled.isCancelled();
   }

   public void start() throws IOException
   {
      packetCommunicator.connect();

      reaMessager.startMessager();

      if (scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(this::mainUpdate, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(bufferUpdater.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }

   public void stop()
   {
      packetCommunicator.closeConnection();
      packetCommunicator.close();

      reaMessager.closeMessager();

      if (scheduled != null)
      {
         scheduled.cancel(true);
         scheduled = null;
      }

      if (executorService != null)
      {
         executorService.shutdownNow();
         executorService = null;
      }
   }

   public static LIDARBasedREAModule createRemoteREAModule()
   {
      REAMessager server = REAMessagerOverNetwork.createServer(NetworkPorts.REA_MODULE_UI_PORT, new REACommunicationKryoNetClassList());
      return new LIDARBasedREAModule(server);
   }

   public static LIDARBasedREAModule createIntraprocessModule()
   {
      REAMessager messager = new REAMessagerSharedVariables();
      return new LIDARBasedREAModule(messager);
   }
}
