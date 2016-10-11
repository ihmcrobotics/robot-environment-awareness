package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.lang3.time.StopWatch;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegion;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionConcaveHull;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionConvexPolygons;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionIntersectionCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.time.TimeTools;

public class REAPlanarRegionFeatureUpdater implements RegionFeaturesProvider
{
   private static final boolean REPORT_TIME = true;
   private final StopWatch stopWatch = REPORT_TIME ? new StopWatch() : null;

   private final NormalOcTree octree;

   private final PlanarRegionCalculator planarRegionCalculator = new PlanarRegionCalculator();
   private final PlanarRegionIntersectionCalculator intersectionCalculator = new PlanarRegionIntersectionCalculator();
   
   private final TIntArrayList regionIds = new TIntArrayList();
   private Map<PlanarRegion, PlanarRegionConcaveHull> concaveHulls = null;
   private Map<PlanarRegion, PlanarRegionConvexPolygons> convexPolygons = null;

   private final AtomicReference<Boolean> isOcTreeEnabled;
   private final AtomicReference<Boolean> enableSegmentation;
   private final AtomicReference<Boolean> clearSegmentation;
   private final AtomicReference<Boolean> enablePolygonizer;
   private final AtomicReference<Boolean> enableIntersectionCalulator;
   private final AtomicReference<PlanarRegionSegmentationParameters> planarRegionSegmentationParameters;
   private final AtomicReference<IntersectionEstimationParameters> intersectionEstimationParameters;
   private final AtomicReference<PolygonizerParameters> polygonizerParameters;

   public REAPlanarRegionFeatureUpdater(NormalOcTree octree, REAMessageManager inputManager, REAMessager outputMessager)
   {
      this.octree = octree;

      isOcTreeEnabled = inputManager.createInput(REAModuleAPI.OcTreeEnable);
      enableSegmentation = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionSegmentationEnable);
      clearSegmentation = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionSegmentationClear);
      enablePolygonizer = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerEnable);
      enableIntersectionCalulator = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionEnable);
      planarRegionSegmentationParameters = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionSegmentationParameters, new PlanarRegionSegmentationParameters());
      intersectionEstimationParameters = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionParameters);
      polygonizerParameters = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerParameters, new PolygonizerParameters());
   }

   public void update()
   {
      regionIds.reset();
      intersectionCalculator.clear();

      if (shouldClearSegmentation())
      {
         planarRegionCalculator.clear();
         return;
      }

      if (!isSegmentationEnabled())
      {
         planarRegionCalculator.removeDeadNodes();
         return;
      }

      updateSegmentation();
      updatePolygons();
      updateIntersections();
   }

   private void updateSegmentation()
   {
      if (REPORT_TIME)
      {
         stopWatch.reset();
         stopWatch.start();
      }

      NormalOcTreeNode root = octree.getRoot();
      OcTreeBoundingBoxInterface boundingBox = octree.getBoundingBox();
      PlanarRegionSegmentationParameters parameters = planarRegionSegmentationParameters.get();
      planarRegionCalculator.compute(root, boundingBox, parameters);

      if (REPORT_TIME)
      {
         stopWatch.stop();
         System.out.println("Segmentation took: " + TimeTools.nanoSecondstoSeconds(stopWatch.getNanoTime()));
      }
   }

   private void updateIntersections()
   {
      if (!isIntersectionCalulatorEnabled())
         return;

      if (REPORT_TIME)
      {
         stopWatch.reset();
         stopWatch.start();
      }

      if (intersectionEstimationParameters.get() != null)
         intersectionCalculator.setParameters(intersectionEstimationParameters.getAndSet(null));
      intersectionCalculator.compute(planarRegionCalculator.getPlanarRegions());

      if (REPORT_TIME)
      {
         stopWatch.stop();
         System.out.println("Processing intersections took: " + TimeTools.nanoSecondstoSeconds(stopWatch.getNanoTime()));
      }
   }

   private void updatePolygons()
   {
      if (!isPolygonizerEnabled())
         return;

      if (REPORT_TIME)
      {
         stopWatch.reset();
         stopWatch.start();
      }

      PolygonizerParameters parameters = polygonizerParameters.get();
      List<PlanarRegion> planarRegions = planarRegionCalculator.getPlanarRegions();

      concaveHulls = PlanarRegionPolygonizer.computeConcaveHulls(planarRegions, parameters);
      convexPolygons = PlanarRegionPolygonizer.computeConvexDecomposition(concaveHulls, parameters);

      if (REPORT_TIME)
      {
         stopWatch.stop();
         System.out.println("Processing polygons took: " + TimeTools.nanoSecondstoSeconds(stopWatch.getNanoTime()));
      }
   }

   @Override
   public List<PlanarRegion> getPlanarRegions()
   {
      return planarRegionCalculator.getPlanarRegions();
   }

   @Override
   public boolean hasPolygonizedPlanarRegions()
   {
      return concaveHulls != null;
   }

   @Override
   public PlanarRegionConcaveHull getPlanarRegionConcaveHull(PlanarRegion planarRegion)
   {
      return concaveHulls == null ? null : concaveHulls.get(planarRegion);
   }

   @Override
   public PlanarRegionConvexPolygons getPlanarRegionConvexPolygons(PlanarRegion planarRegion)
   {
      return convexPolygons == null ? null : convexPolygons.get(planarRegion);
   }

   @Override
   public int getNumberOfPlaneIntersections()
   {
      return intersectionCalculator.getNumberOfIntersections();
   }

   @Override
   public LineSegment3d getIntersection(int index)
   {
      return intersectionCalculator.getIntersection(index);
   }

   private boolean isSegmentationEnabled()
   {
      if (!isOcTreeEnabled())
         return false;
      else
         return enableSegmentation.get() == null ? false : enableSegmentation.get();
   }

   private boolean isPolygonizerEnabled()
   {
      if (!isOcTreeEnabled())
         return false;
      else
         return enablePolygonizer.get() == null ? false : enablePolygonizer.get();
   }

   private boolean isIntersectionCalulatorEnabled()
   {
      if (!isOcTreeEnabled())
         return false;
      else
         return enableIntersectionCalulator.get() == null ? false : enableIntersectionCalulator.get();
   }

   public boolean isOcTreeEnabled()
   {
      return isOcTreeEnabled.get() == null ? false : isOcTreeEnabled.get();
   }

   public boolean shouldClearSegmentation()
   {
      return clearSegmentation.get() == null ? false : clearSegmentation.getAndSet(null);
   }
}
