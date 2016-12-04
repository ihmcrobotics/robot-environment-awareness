package us.ihmc.robotEnvironmentAwareness.updaters;

import java.io.File;
import java.io.IOException;
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
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.tools.io.printing.PrintTools;

public class LIDARBasedREAModule
{
   private static final String ocTreeTimeReport = "OcTree update took: ";
   private static final String reportOcTreeStateTimeReport = "Reporting OcTree state took: ";
   private static final String planarRegionsTimeReport = "OcTreePlanarRegion update took: ";
   private static final String reportPlanarRegionsStateTimeReport = "Reporting Planar Regions state took: ";

   private final TimeReporter timeReporter = new TimeReporter(this);

   private static final int THREAD_PERIOD_MILLISECONDS = 200;
   private static final int BUFFER_THREAD_PERIOD_MILLISECONDS = 10;
   private static final double OCTREE_RESOLUTION = 0.02;
   protected static final boolean DEBUG = true;

   private final String networkManagerHost = NetworkParameters.getHost(NetworkParameterKeys.networkManager);
   private final PacketCommunicator packetCommunicator;

   private final NormalOcTree mainOctree = new NormalOcTree(OCTREE_RESOLUTION);

   private final REAOcTreeBuffer bufferUpdater;
   private final REAOcTreeUpdater mainUpdater;
   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdater;

   private final REAModuleStateReporter moduleStateReporter;
   private final REAPlanarRegionNetworkProvider planarRegionNetworkProvider;

   private final AtomicReference<Boolean> clearOcTree;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> scheduled;
   private final REAMessager reaMessager;

   private LIDARBasedREAModule(REAMessager reaMessager, File configurationFile) throws IOException
   {
      this.reaMessager = reaMessager;

      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(networkManagerHost, NetworkPorts.REA_MODULE_PORT, new IHMCCommunicationKryoNetClassList());
      packetCommunicator.connect();

      timeReporter.enableTimeReport(true);

      moduleStateReporter = new REAModuleStateReporter(reaMessager, packetCommunicator);

      bufferUpdater = new REAOcTreeBuffer(mainOctree.getResolution(), reaMessager, moduleStateReporter, packetCommunicator);
      mainUpdater = new REAOcTreeUpdater(mainOctree, bufferUpdater, reaMessager, packetCommunicator);
      planarRegionFeatureUpdater = new REAPlanarRegionFeatureUpdater(mainOctree, reaMessager);

      FilePropertyHelper filePropertyHelper = new FilePropertyHelper(configurationFile);
      loadConfigurationFile(filePropertyHelper);

      reaMessager.registerTopicListener(REAModuleAPI.SaveBufferConfiguration, (content) -> bufferUpdater.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveMainUpdaterConfiguration, (content) -> mainUpdater.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveRegionUpdaterConfiguration, (content) -> planarRegionFeatureUpdater.saveConfiguration(filePropertyHelper));

      planarRegionNetworkProvider = new REAPlanarRegionNetworkProvider(planarRegionFeatureUpdater, packetCommunicator);
      clearOcTree = reaMessager.createInput(REAModuleAPI.OcTreeClear, false);
   }

   private void loadConfigurationFile(FilePropertyHelper filePropertyHelper)
   {
      bufferUpdater.loadConfiguration(filePropertyHelper);
      mainUpdater.loadConfiguration(filePropertyHelper);
      planarRegionFeatureUpdater.loadConfiguration(filePropertyHelper);
   }

   private final AtomicDouble lastCompleteUpdate = new AtomicDouble(Double.NaN);

   private void mainUpdate()
   {
      if (isThreadInterrupted())
         return;

      double currentTime = JOctoMapTools.nanoSecondsToSeconds(System.nanoTime());

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
            timeReporter.run(() -> moduleStateReporter.reportOcTreeState(mainOctree), reportOcTreeStateTimeReport);

            if (isThreadInterrupted())
               return;

            timeReporter.run(planarRegionFeatureUpdater::update, planarRegionsTimeReport);
            timeReporter.run(() -> moduleStateReporter.reportPlanarRegionsState(planarRegionFeatureUpdater), reportPlanarRegionsStateTimeReport);

            planarRegionNetworkProvider.update(ocTreeUpdateSuccess);
         }

         if (isThreadInterrupted())
            return;

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
   }

   private boolean isThreadInterrupted()
   {
      return Thread.interrupted() || scheduled == null || scheduled.isCancelled();
   }

   public void start() throws IOException
   {
      if (scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(this::mainUpdate, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(bufferUpdater.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }

   public void stop()
   {
      PrintTools.info("REA Module is going down.");
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

   public static LIDARBasedREAModule createRemoteModule(String configurationFilePath) throws IOException
   {
      REAMessager server = REAMessagerOverNetwork.createTCPServer(NetworkPorts.REA_MODULE_UI_PORT, new REACommunicationKryoNetClassList());
      server.startMessager();
      return new LIDARBasedREAModule(server, new File(configurationFilePath));
   }

   public static LIDARBasedREAModule createIntraprocessModule(String configurationFilePath) throws IOException
   {
      REAMessager messager = REAMessagerOverNetwork.createIntraprocess(NetworkPorts.REA_MODULE_UI_PORT, new REACommunicationKryoNetClassList());
      messager.startMessager();
      return new LIDARBasedREAModule(messager, new File(configurationFilePath));
   }
}
