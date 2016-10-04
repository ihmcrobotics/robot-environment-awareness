package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.octoMap.ocTree.implementations.NormalOcTree;
import us.ihmc.octoMap.tools.OctoMapTools;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class LIDARBasedREAModule
{
   private static final boolean REPORT_TIME = false;
   private static final int THREAD_PERIOD_MILLISECONDS = 500;
   private static final double GRAPHICS_REFRESH_PERIOD = 0.25; // in seconds
   private static final double OCTREE_RESOLUTION = 0.02;
   protected static final boolean DEBUG = true;

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
         private final AtomicDouble lastGraphicsUpdate = new AtomicDouble(Double.NaN);

         @Override
         public void run()
         {
            if (isThreadInterrupted())
               return;

            try
            {
               if (REPORT_TIME)
               {
                  long startTime = System.nanoTime();
                  updater.update();
                  long endTime = System.nanoTime();
                  System.out.println("OcTree update took: " + OctoMapTools.nanoSecondsToSeconds(endTime - startTime));
               }
               else
               {
                  updater.update();
               }

               if (isThreadInterrupted())
                  return;

               planarRegionFeatureUpdater.update();

               if (isThreadInterrupted())
                  return;

               double currentTime = OctoMapTools.nanoSecondsToSeconds(System.nanoTime());

               if (Double.isNaN(lastGraphicsUpdate.get()) || currentTime - lastGraphicsUpdate.get() >= GRAPHICS_REFRESH_PERIOD)
               {
                  lastGraphicsUpdate.set(currentTime);
                  if (REPORT_TIME)
                  {
                     long startTime = System.nanoTime();
                     graphicsBuilder.update();
                     long endTime = System.nanoTime();
                     System.out.println("OcTreeGraphics update took: " + OctoMapTools.nanoSecondsToSeconds(endTime - startTime));
                  }
                  else
                  {
                     graphicsBuilder.update();
                  }
               }
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
