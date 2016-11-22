package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.lang3.time.StopWatch;

import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.tools.JOctoMapTools;

import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
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

   private final StopWatch stopWatch = REPORT_TIME ? new StopWatch() : null;
   private final NormalOcTree octree = new NormalOcTree(OCTREE_RESOLUTION);

   private final REAOcTreeUpdater updater;
   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdater;

   private final REAOcTreeGraphicsBuilder graphicsBuilder;

   private final REAPlanarRegionNetworkProvider planarRegionNetworkProvider;

   private final AtomicReference<Boolean> clearOcTree;

   private ScheduledExecutorService executorService = Executors.newScheduledThreadPool(3, ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private ScheduledFuture<?> scheduled;

   public LIDARBasedREAModule(REAMessager reaMessager)
   {
      updater = new REAOcTreeUpdater(octree, reaMessager);

      planarRegionFeatureUpdater = new REAPlanarRegionFeatureUpdater(octree, reaMessager);

      // TODO move Graphics builder to UI
      graphicsBuilder = new REAOcTreeGraphicsBuilder(octree, planarRegionFeatureUpdater, reaMessager);

      // lui donner le messager
      planarRegionNetworkProvider = new REAPlanarRegionNetworkProvider(planarRegionFeatureUpdater);
      clearOcTree = reaMessager.createInput(REAModuleAPI.OcTreeClear, false);
   }

   public void attachListeners(PacketCommunicator packetCommunicator)
   {
      updater.attachListeners(packetCommunicator);
      planarRegionNetworkProvider.attachPacketCommunicator(packetCommunicator);
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

            return updatedProperly;
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

            // TODO: Here Send stuff to GUI

            Runnable futureTask = graphicsBuilder.update();
            if (futureTask != null)
               executorService.execute(futureTask);

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

   public void start()
   {
      if (scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(createUpdater(), 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(updater.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }

   public void stop()
   {
      if (scheduled != null)
      {
         scheduled.cancel(true);
         scheduled = null;
      }

      if (executorService != null)
      {
         executorService.shutdown();
         executorService = null;
      }
   }
}
