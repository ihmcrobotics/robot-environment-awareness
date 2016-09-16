package us.ihmc.robotEnvironmentAwareness.ui.ocTree;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import com.google.common.util.concurrent.AtomicDouble;

import javafx.scene.paint.Material;
import javafx.scene.shape.Box;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.octoMap.ocTree.implementations.NormalOcTree;
import us.ihmc.octoMap.occupancy.OccupancyParameters;
import us.ihmc.octoMap.tools.OctoMapTools;
import us.ihmc.robotEnvironmentAwareness.ui.ocTree.OcTreeGraphicsBuilder.ColoringType;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class OcTreeUIController
{
   private static final int THREAD_PERIOD_MILLISECONDS = 3000;
   private static final double GRAPHICS_REFRESH_PERIOD = 1.0; // in seconds
   private static final double OCTREE_RESOLUTION = 0.01;
   protected static final boolean DEBUG = true;

   private final NormalOcTree octree = new NormalOcTree(OCTREE_RESOLUTION);

   private final AtomicReference<OccupancyParameters> atomicOccupancyParameters = new AtomicReference<OccupancyParameters>(new OccupancyParameters());

   private final OcTreeUpdater updater;
   private final OcTreeGraphicsBuilder graphicsBuilder;

   private ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private ScheduledFuture<?> scheduled;

   public OcTreeUIController(boolean enableInitialValue)
   {
      updater = new OcTreeUpdater(octree, enableInitialValue);
      graphicsBuilder = new OcTreeGraphicsBuilder(octree, enableInitialValue);

      updater.setTreeDepthUsedForDisplay(graphicsBuilder.getCurrentTreeDepthForDisplay());
   }

   public void attachListeners(PacketCommunicator packetCommunicator)
   {
      updater.attachListeners(packetCommunicator);
   }

   public void setOcTreeOccupancyParameters(OccupancyParameters occupancyParameters)
   {
      atomicOccupancyParameters.set(occupancyParameters);
   }

   public OccupancyParameters getOcTreeOccupancyParameters()
   {
      return atomicOccupancyParameters.get();
   }

   public void setEnable(boolean enable)
   {
      updater.setEnable(enable);
      graphicsBuilder.setEnable(enable);
   }

   public void clear()
   {
      updater.clear();
      graphicsBuilder.clear();
   }

   public boolean hasProcessedClear()
   {
      return updater.hasProcessedClear() && graphicsBuilder.hasProcessedClear();
   }

   public void setMinRange(double minRange)
   {
      updater.setMinRange(minRange);
   }

   public double getMinRange()
   {
      return updater.getMinRange();
   }

   public void setMaxRange(double maxRange)
   {
      updater.setMaxRange(maxRange);
   }

   public double getMaxRange()
   {
      return updater.getMaxRange();
   }

   public boolean isBoundingBoxEnabled()
   {
      return updater.isBoundingBoxEnabled();
   }

   public void enableBoundingBox(boolean enable)
   {
      updater.enableBoundingBox(enable);
   }

   public OcTreeBoundingBoxWithCenterAndYaw getBoundingBox()
   {
      return updater.getBoundingBox();
   }

   public void setBoundingBox(OcTreeBoundingBoxWithCenterAndYaw boundingBox)
   {
      updater.setBoundingBox(boundingBox);
   }

   public void setTreeDepthForDisplay(int newDepth)
   {
      graphicsBuilder.setTreeDepthForDisplay(newDepth);
      updater.setTreeDepthUsedForDisplay(newDepth);
   }

   public int getCurrentTreeDepthForDisplay()
   {
      return graphicsBuilder.getCurrentTreeDepthForDisplay();
   }

   public void showFreeSpace(boolean show)
   {
      graphicsBuilder.showFreeSpace(show);
   }

   public boolean isShowingFreeSpace()
   {
      return graphicsBuilder.isShowingFreeSpace();
   }

   public void setColoringType(ColoringType type)
   {
      graphicsBuilder.setColoringType(type);
   }

   public ColoringType getColoringType()
   {
      return graphicsBuilder.getColoringType();
   }

   public void showEstimatedSurfaces(boolean show)
   {
      graphicsBuilder.showEstimatedSurfaces(show);
   }

   public boolean isShowingEstimatedSurfaces()
   {
      return graphicsBuilder.isShowingEstimatedSurfaces();
   }

   public boolean hasNewOccupiedMeshToRender()
   {
      return graphicsBuilder.hasNewOccupiedMeshToRender();
   }

   public boolean hasNewFreeMeshToRender()
   {
      return graphicsBuilder.hasNewFreeMeshToRender();
   }

   public Pair<Mesh, Material> pollOccupiedMesh()
   {
      return graphicsBuilder.pollOccupiedMesh();
   }

   public Pair<Mesh, Material> pollFreeMesh()
   {
      return graphicsBuilder.pollFreeMesh();
   }

   public void showOcTreeBoundingBox(boolean show)
   {
      graphicsBuilder.showOcTreeBoundingBox(show);
   }

   public boolean isShowingOcTreeBoundingBox()
   {
      return graphicsBuilder.isShowingOcTreeBoundingBox();
   }

   public boolean hasNewOcTreeBoundingBoxToRender()
   {
      return graphicsBuilder.hasNewOcTreeBoundingBoxToRender();
   }

   public Box pollOcTreeBoundingBoxGraphics()
   {
      return graphicsBuilder.pollOcTreeBoundingBoxGraphics();
   }

   public int getMaximumTreeDepth()
   {
      return octree.getTreeDepth();
   }

   private Runnable createUpdater()
   {
      Runnable runnable = new Runnable()
      {
         private final AtomicDouble lastGraphicsUpdate = new AtomicDouble(Double.NaN);

         @Override
         public void run()
         {
            if (Thread.interrupted())
               return;

            try
            {
               updateOcTreeSettings();
               updater.update();

               if (Thread.interrupted())
                  return;

               double currentTime = OctoMapTools.nanoSecondsToSeconds(System.nanoTime());

               if (Double.isNaN(lastGraphicsUpdate.get()) || currentTime - lastGraphicsUpdate.get() >= GRAPHICS_REFRESH_PERIOD)
               {
                  lastGraphicsUpdate.set(currentTime);
                  graphicsBuilder.update();
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
                  PrintTools.error(OcTreeUIController.class, e.getClass().getSimpleName());
               }
            }
         }
      };
      return runnable;
   }

   private void updateOcTreeSettings()
   {
      OccupancyParameters occupancyParameters = atomicOccupancyParameters.get();
      if (occupancyParameters != null)
         octree.setOccupancyParameters(occupancyParameters);
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
