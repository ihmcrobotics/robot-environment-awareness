package us.ihmc.robotEnvironmentAwareness.updaters;

import java.io.IOException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.lang3.time.StopWatch;

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
   private static final boolean REPORT_TIME = false;
   private static final int THREAD_PERIOD_MILLISECONDS = 200;
   private static final int BUFFER_THREAD_PERIOD_MILLISECONDS = 10;
   private static final double OCTREE_COMPLETE_UPDATE_PERIOD = 0.5; // in seconds
   private static final double GRAPHICS_REFRESH_PERIOD = 2.0; // in seconds
   private static final double OCTREE_RESOLUTION = 0.02;
   protected static final boolean DEBUG = true;

   private final String networkManagerHost = NetworkParameters.getHost(NetworkParameterKeys.networkManager);
   private final PacketCommunicator packetCommunicator;

   private final StopWatch stopWatch = REPORT_TIME ? new StopWatch() : null;
   private final NormalOcTree octree = new NormalOcTree(OCTREE_RESOLUTION);

   private final REAOcTreeBuffer buffer;
   private final REAOcTreeUpdater updater;
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
      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(networkManagerHost, NetworkPorts.REA_MODULE_PORT, new IHMCCommunicationKryoNetClassList());

      buffer = new REAOcTreeBuffer(octree.getResolution(), reaMessager, packetCommunicator);
      updater = new REAOcTreeUpdater(octree, buffer, reaMessager, packetCommunicator);

      planarRegionFeatureUpdater = new REAPlanarRegionFeatureUpdater(octree, reaMessager);
      graphicsBuilder = new REAOcTreeGraphicsBuilder(octree, planarRegionFeatureUpdater, reaMessager);

      planarRegionNetworkProvider = new REAPlanarRegionNetworkProvider(planarRegionFeatureUpdater, packetCommunicator);
      clearOcTree = reaMessager.createInput(REAModuleAPI.OcTreeClear, false);
   }

   private Runnable createUpdater()
   {
      Runnable runnable = new Runnable()
      {
         private final AtomicDouble lastCompleteUpdate = new AtomicDouble(Double.NaN);
         private final AtomicDouble lastGraphicsUpdate = new AtomicDouble(Double.NaN);

         @Override
         public void run()
         {
            if (isThreadInterrupted())
               return;

            double currentTime = JOctoMapTools.nanoSecondsToSeconds(System.nanoTime());

            boolean performCompleteOcTreeUpdate = (Double.isNaN(lastCompleteUpdate.get()) || currentTime - lastCompleteUpdate.get() >= OCTREE_COMPLETE_UPDATE_PERIOD);
            boolean performGraphicsUpdate = (Double.isNaN(lastGraphicsUpdate.get()) || currentTime - lastGraphicsUpdate.get() >= GRAPHICS_REFRESH_PERIOD);

            try
            {
               if (clearOcTree.getAndSet(false))
               {
                  buffer.clearBuffer();
                  updater.clearOcTree();
                  planarRegionFeatureUpdater.clearOcTree();
               }
               else
               {
                  performCompleteOcTreeUpdate &= callOcTreeUpdater(performCompleteOcTreeUpdate);

                  if (isThreadInterrupted())
                     return;

                  if (performCompleteOcTreeUpdate)
                     callPlanarRegionFeatureUpdater();
               }

               if (isThreadInterrupted())
                  return;

               if (performGraphicsUpdate)
                  callGraphicsBuilder();

               planarRegionNetworkProvider.update(performCompleteOcTreeUpdate);

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
            if (performCompleteOcTreeUpdate)
               lastCompleteUpdate.set(currentTime);
            if (performGraphicsUpdate)
               lastGraphicsUpdate.set(currentTime);
         }

         private boolean callOcTreeUpdater(boolean performCompleteUpdate)
         {
            if (REPORT_TIME)
            {
               if (performCompleteUpdate)
               {
                  stopWatch.reset();
                  stopWatch.start();
               }
            }

            boolean updatedProperly = updater.update(performCompleteUpdate);

            if (REPORT_TIME)
            {
               if (performCompleteUpdate && updatedProperly)
                  System.out.println("OcTree update took: " + JOctoMapTools.nanoSecondsToSeconds(stopWatch.getNanoTime()));
            }

            return performCompleteUpdate && updatedProperly;
         }

         private void callPlanarRegionFeatureUpdater()
         {
            if (REPORT_TIME)
            {
               stopWatch.reset();
               stopWatch.start();
            }

            planarRegionFeatureUpdater.update();

            if (REPORT_TIME)
            {
               System.out.println("OcTreePlanarRegion update took: " + JOctoMapTools.nanoSecondsToSeconds(stopWatch.getNanoTime()));
            }
         }

         private void callGraphicsBuilder()
         {
            if (REPORT_TIME)
            {
               stopWatch.reset();
               stopWatch.start();
            }

            graphicsBuilder.update();

            if (REPORT_TIME)
            {
               System.out.println("OcTreeGraphics update took: " + JOctoMapTools.nanoSecondsToSeconds(stopWatch.getNanoTime()));
            }
         }

         private boolean isThreadInterrupted()
         {
            return Thread.interrupted() || scheduled == null || scheduled.isCancelled();
         }
      };
      return runnable;
   }

   public void start() throws IOException
   {
      packetCommunicator.connect();

      reaMessager.startMessager();

      if (scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(createUpdater(), 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(buffer.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
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
