package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import org.apache.commons.lang3.time.StopWatch;

import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.tools.OctoMapTools;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class LIDARBasedREAModule
{
   private static final boolean REPORT_TIME = false;
   private static final int THREAD_PERIOD_MILLISECONDS = 50;
   private static final double OCTREE_COMPLETE_UPDATE_PERIOD = 0.5; // in seconds
   private static final double GRAPHICS_REFRESH_PERIOD = 0.5; // in seconds
   private static final double OCTREE_RESOLUTION = 0.02;
   protected static final boolean DEBUG = true;

   private final StopWatch stopWatch = REPORT_TIME ? new StopWatch() : null;
   private final NormalOcTree octree = new NormalOcTree(OCTREE_RESOLUTION);

   private final REAOcTreeUpdater updater;
   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdater;
   private final REAOcTreeGraphicsBuilder graphicsBuilder;

   private ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private ScheduledFuture<?> scheduled;

   public LIDARBasedREAModule(REAMessageManager uiOutputManager, REAMessager uiInputMessager)
   {
      updater = new REAOcTreeUpdater(octree, uiOutputManager, uiInputMessager);
      planarRegionFeatureUpdater = new REAPlanarRegionFeatureUpdater(octree, uiOutputManager, uiInputMessager);
      graphicsBuilder = new REAOcTreeGraphicsBuilder(octree, planarRegionFeatureUpdater, uiOutputManager, uiInputMessager);
   }

   public void attachListeners(PacketCommunicator packetCommunicator)
   {
      updater.attachListeners(packetCommunicator);
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


            double currentTime = OctoMapTools.nanoSecondsToSeconds(System.nanoTime());
            
            boolean performCompleteOcTreeUpdate = (Double.isNaN(lastCompleteUpdate.get()) || currentTime - lastCompleteUpdate.get() >= OCTREE_COMPLETE_UPDATE_PERIOD);
            boolean performGraphicsUpdate = (Double.isNaN(lastGraphicsUpdate.get()) || currentTime - lastGraphicsUpdate.get() >= GRAPHICS_REFRESH_PERIOD);

            try
            {
               performCompleteOcTreeUpdate &= callOcTreeUpdater(performCompleteOcTreeUpdate);

               if (isThreadInterrupted())
                  return;

               if (performCompleteOcTreeUpdate)
                  planarRegionFeatureUpdater.update();

               if (isThreadInterrupted())
                  return;


               if (performGraphicsUpdate)
                  callGraphicsBuilder();

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

            currentTime = OctoMapTools.nanoSecondsToSeconds(System.nanoTime());
            if (performCompleteOcTreeUpdate)
               lastCompleteUpdate.set(currentTime);
            if (performGraphicsUpdate)
               lastGraphicsUpdate.set(currentTime);
         }

         private boolean callOcTreeUpdater(boolean performCompleteUpdate)
         {
            if (REPORT_TIME)
            {
               stopWatch.reset();
               stopWatch.start();
            }

            boolean updatedProperly = updater.update(performCompleteUpdate);

            if (REPORT_TIME)
            {
               System.out.println("OcTree update took: " + OctoMapTools.nanoSecondsToSeconds(stopWatch.getNanoTime()));
            }

            return updatedProperly;
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
               System.out.println("OcTreeGraphics update took: " + OctoMapTools.nanoSecondsToSeconds(stopWatch.getNanoTime()));
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
         scheduled = executorService.scheduleAtFixedRate(createUpdater(), 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
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
