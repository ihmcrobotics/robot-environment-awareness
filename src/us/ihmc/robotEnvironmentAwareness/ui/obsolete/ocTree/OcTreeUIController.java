package us.ihmc.robotEnvironmentAwareness.ui.obsolete.ocTree;

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
import us.ihmc.octoMap.ocTree.implementations.NormalEstimationParameters;
import us.ihmc.octoMap.ocTree.implementations.NormalOcTree;
import us.ihmc.octoMap.ocTree.implementations.PlanarRegionSegmentationParameters;
import us.ihmc.octoMap.occupancy.OccupancyParameters;
import us.ihmc.octoMap.tools.OctoMapTools;
import us.ihmc.robotEnvironmentAwareness.ui.obsolete.ocTree.OcTreeGraphicsBuilder.ColoringType;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class OcTreeUIController
{
   private static final boolean REPORT_TIME = false;
   private static final int THREAD_PERIOD_MILLISECONDS = 500;
   private static final double GRAPHICS_REFRESH_PERIOD = 0.25; // in seconds
   private static final double OCTREE_RESOLUTION = 0.02;
   protected static final boolean DEBUG = true;

   private final NormalOcTree octree = new NormalOcTree(OCTREE_RESOLUTION);

   private final AtomicReference<OccupancyParameters> atomicOccupancyParameters = new AtomicReference<>(new OccupancyParameters());
   private final AtomicReference<NormalEstimationParameters> atomicNormalEstimationParameters = new AtomicReference<>(new NormalEstimationParameters());
   private final AtomicReference<PlanarRegionSegmentationParameters> atomicPlanarRegionSegmentationParameters = new AtomicReference<>(new PlanarRegionSegmentationParameters());

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

   public void setOcTreeOccupancyParameters(OccupancyParameters parameters)
   {
      atomicOccupancyParameters.set(parameters);
   }

   public OccupancyParameters getOcTreeOccupancyParameters()
   {
      return atomicOccupancyParameters.get();
   }

   public void setOcTreeNormalEstimationParameters(NormalEstimationParameters parameters)
   {
      atomicNormalEstimationParameters.set(parameters);
   }

   public NormalEstimationParameters getOcTreeNormalEstimationParameters()
   {
      return atomicNormalEstimationParameters.get();
   }

   public void setOcTreePlanarRegionSegmentationParameters(PlanarRegionSegmentationParameters parameters)
   {
      atomicPlanarRegionSegmentationParameters.set(parameters);
   }

   public PlanarRegionSegmentationParameters getOcTreePlanarRegionSegmentationParameters()
   {
      return atomicPlanarRegionSegmentationParameters.get();
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

   public void showPlanarRegions(boolean show)
   {
      graphicsBuilder.showPlanarRegions(show);
   }

   public boolean isShowingPlanarRegions()
   {
      return graphicsBuilder.isShowingPlanarRegions();
   }

   public void hidePlanarRegionNodes(boolean show)
   {
      graphicsBuilder.hidePlanarRegionNodes(show);
   }

   public boolean isHidingPlanarRegionNodes()
   {
      return graphicsBuilder.isHidingPlanarRegionNodes();
   }

   public boolean hasNewOccupiedMeshToRender()
   {
      return graphicsBuilder.hasNewOccupiedMeshToRender();
   }

   public boolean hasNewFreeMeshToRender()
   {
      return graphicsBuilder.hasNewFreeMeshToRender();
   }

   public boolean hasNewPlanarRegionPolygonMeshToRender()
   {
      return graphicsBuilder.hasNewPlanarRegionPolygonMeshToRender();
   }

   public Pair<Mesh, Material> pollOccupiedMesh()
   {
      return graphicsBuilder.pollOccupiedMesh();
   }

   public Pair<Mesh, Material> pollFreeMesh()
   {
      return graphicsBuilder.pollFreeMesh();
   }

   public Pair<Mesh, Material> pollPlanarRegionPolygonMesh()
   {
      return graphicsBuilder.pollPlanarRegionPolygonMesh();
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
               if (REPORT_TIME)
               {
                  updateOcTreeSettings();
                  long startTime = System.nanoTime();
                  updater.update();
                  long endTime = System.nanoTime();
                  System.out.println("OcTree update took: " + OctoMapTools.nanoSecondsToSeconds(endTime - startTime));
               }
               else
               {
                  updater.update();
               }

               if (Thread.interrupted())
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
      NormalEstimationParameters normalEstimationParameters = atomicNormalEstimationParameters.get();
      if (normalEstimationParameters != null)
         octree.setNormalEstimationParameters(normalEstimationParameters);
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = atomicPlanarRegionSegmentationParameters.get();
      if (planarRegionSegmentationParameters != null)
         octree.setPlanarRegionSegmentationParameters(planarRegionSegmentationParameters);
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
