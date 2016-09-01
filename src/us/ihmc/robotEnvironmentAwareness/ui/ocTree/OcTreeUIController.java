package us.ihmc.robotEnvironmentAwareness.ui.ocTree;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import com.google.common.util.concurrent.AtomicDouble;

import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.octoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.ocTree.OcTreeGraphicsBuilder.ColoringType;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class OcTreeUIController
{
   private static final int THREAD_PERIOD_MILLISECONDS = 100;
   private static final double OCTREE_RESOLUTION = 0.025;
   protected static final boolean DEBUG = false;

   private static final double DEFAULT_HIT_UPDATE = 0.7;
   private static final double DEFAULT_MISS_UPDATE = 0.4;
   private static final double DEFAULT_MIN_PROBABILITY = 0.1192;
   private static final double DEFAULT_MAX_PROBABILITY = 0.971;
   private static final double DEFAULT_OCCUPANCY_THRESHOLD = 0.5;

   private final NormalOcTree octree = new NormalOcTree(OCTREE_RESOLUTION);

   private final AtomicDouble hitUpdate = new AtomicDouble(DEFAULT_HIT_UPDATE);
   private final AtomicDouble missUpdate = new AtomicDouble(DEFAULT_MISS_UPDATE);
   private final AtomicDouble minProbability = new AtomicDouble(DEFAULT_MIN_PROBABILITY);
   private final AtomicDouble maxProbability = new AtomicDouble(DEFAULT_MAX_PROBABILITY);
   private final AtomicDouble occupancyThreshold = new AtomicDouble(DEFAULT_OCCUPANCY_THRESHOLD);

   private final OcTreeUpdater updater;
   private final OcTreeGraphicsBuilder graphicsBuilder;

   private ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private ScheduledFuture<?> scheduled;

   public OcTreeUIController(boolean enableInitialValue)
   {
      updater = new OcTreeUpdater(octree, enableInitialValue);
      graphicsBuilder = new OcTreeGraphicsBuilder(octree, enableInitialValue);
   }

   public void attachListeners(PacketCommunicator packetCommunicator)
   {
      updater.attachListeners(packetCommunicator);
   }

   public void setOcTreeOccupancyThreshold(double probabilityThreshold)
   {
      if (Double.isNaN(probabilityThreshold))
         occupancyThreshold.set(DEFAULT_OCCUPANCY_THRESHOLD);
      else if (MathTools.isInsideBoundsExclusive(probabilityThreshold, 0.0, 1.0))
         occupancyThreshold.set(probabilityThreshold);
   }

   public double getOcTreeOccupancyThreshold()
   {
      return occupancyThreshold.get();
   }

   public void setOcTreeHitUpdate(double probabilityUpdate)
   {
      if (Double.isNaN(probabilityUpdate))
         hitUpdate.set(DEFAULT_HIT_UPDATE);
      else if (MathTools.isInsideBoundsExclusive(probabilityUpdate, 0.0, 1.0))
         hitUpdate.set(probabilityUpdate);
   }

   public double getOcTreeHitUpdate()
   {
      return hitUpdate.get();
   }

   public void setOcTreeMissUpdate(double probabilityUpdate)
   {
      if (Double.isNaN(probabilityUpdate))
         missUpdate.set(DEFAULT_MISS_UPDATE);
      else if (MathTools.isInsideBoundsExclusive(probabilityUpdate, 0.0, 1.0))
         missUpdate.set(probabilityUpdate);
   }

   public double getOcTreeMissUpdate()
   {
      return missUpdate.get();
   }

   public void setOcTreeMinimumProbability(double minProbability)
   {
      if (Double.isNaN(minProbability))
         this.minProbability.set(DEFAULT_MIN_PROBABILITY);
      else if (MathTools.isInsideBoundsExclusive(minProbability, 0.0, 1.0))
         this.minProbability.set(minProbability);
   }

   public double getOcTreeMinimumProbability()
   {
      return minProbability.get();
   }

   public void setOcTreeMaximumProbability(double maxProbability)
   {
      if (Double.isNaN(maxProbability))
         this.maxProbability.set(DEFAULT_MAX_PROBABILITY);
      else if (MathTools.isInsideBoundsExclusive(maxProbability, 0.0, 1.0))
         this.maxProbability.set(maxProbability);
   }

   public double getOcTreeMaximumProbability()
   {
      return maxProbability.get();
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

   public void setTreeDepthForDisplay(int newDepth)
   {
      graphicsBuilder.setTreeDepthForDisplay(newDepth);
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

   public boolean isBoundingBoxEnabled()
   {
      return graphicsBuilder.isBoundingBoxEnabled();
   }

   public void enableBoundingBox(boolean enable)
   {
      graphicsBuilder.enableBoundingBox(enable);
   }

   public BoundingBox3d getBoundingBox()
   {
      return graphicsBuilder.getBoundingBox();
   }

   public void setBoundingBox(BoundingBox3d boundingBox3d)
   {
      graphicsBuilder.setBoundingBox(boundingBox3d);
   }

   public int getMaximumTreeDepth()
   {
      return octree.getTreeDepth();
   }

   private Runnable createUpdater()
   {
      Runnable runnable = new Runnable()
      {
         @Override
         public void run()
         {
            if (Thread.interrupted())
               return;

            try
            {
               updateOcTreeSettings();
               updater.update();
               graphicsBuilder.update();
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
      octree.setOccupancyThreshold(occupancyThreshold.get());

      octree.setHitProbabilityUpdate(hitUpdate.get());
      octree.setMissProbabilityUpdate(missUpdate.get());

      octree.setMinProbability(minProbability.get());
      octree.setMaxProbability(maxProbability.get());
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
