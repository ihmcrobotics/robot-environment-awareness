package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;

import org.apache.commons.lang3.time.StopWatch;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegion;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionCalculator;
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
   private final PlanarRegionPolygonizer planarRegionPolygonizer = new PlanarRegionPolygonizer();
   
   private int minPlanarRegionSize = 10;

   private final TIntArrayList regionIds = new TIntArrayList();
   private final List<ConcaveHullVertices> concaveHullsVertices = new ArrayList<>();
   private final List<List<ConvexHullVertices>> convexHullsVertices = new ArrayList<>();

   private final AtomicReference<Boolean> isOcTreeEnabled;
   private final AtomicReference<Boolean> enableSegmentation;
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
      enablePolygonizer = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerEnable);
      enableIntersectionCalulator = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionEnable);
      planarRegionSegmentationParameters = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionSegmentationParameters, new PlanarRegionSegmentationParameters());
      intersectionEstimationParameters = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionParameters);
      polygonizerParameters = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerParameters);
   }

   public void update()
   {
      regionIds.reset();
      concaveHullsVertices.clear();
      convexHullsVertices.clear();
      intersectionCalculator.clear();

      if (!isSegmentationEnabled())
         return;

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

      if (polygonizerParameters.get() != null)
         planarRegionPolygonizer.setParameters(polygonizerParameters.getAndSet(null));

      if (REPORT_TIME)
      {
         stopWatch.reset();
         stopWatch.start();
      }

      for (PlanarRegion planarRegion : planarRegionCalculator.getPlanarRegions())
      {
         if (planarRegion.getNumberOfNodes() < minPlanarRegionSize)
            continue;

         regionIds.add(planarRegion.getId());
         planarRegionPolygonizer.compute(planarRegion);
         
         ConcaveHullVertices concaveHullVertices = new ConcaveHullVertices();
         concaveHullVertices.addAll(planarRegionPolygonizer.getConcaveHullVertices());
         concaveHullsVertices.add(concaveHullVertices);

         ArrayList<ConvexHullVertices> regionConvexHullsVertices = new ArrayList<>();
         convexHullsVertices.add(regionConvexHullsVertices);

         for (int j = 0; j < planarRegionPolygonizer.getNumberOfConvexPolygons(); j++)
         {
            ConvexHullVertices convexHullVertices = new ConvexHullVertices();
            convexHullVertices.addAll(planarRegionPolygonizer.getConvexPolygonVertices(j));
            regionConvexHullsVertices.add(convexHullVertices);
         }
      }

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
   public int getNumberOfConcaveHulls()
   {
      return concaveHullsVertices.size();
   }

   @Override
   public int getRegionId(int concaveHullIndex)
   {
      return regionIds.get(concaveHullIndex);
   }

   @Override
   public List<Point3d> getConcaveHull(int concaveHullIndex)
   {
      return concaveHullsVertices.get(concaveHullIndex);
   }
   
   @Override
   public int getNumberOfConvexHulls(int concaveHullIndex)
   {
      return convexHullsVertices.get(concaveHullIndex).size();
   }

   @Override
   public List<Point3d> getConvexHull(int concaveHullIndex, int convexHullIndex)
   {
      return convexHullsVertices.get(concaveHullIndex).get(convexHullIndex);
   }

   @Override
   public int getNumberOfPlaneIntersections()
   {
      return intersectionCalculator.getNumberOfIntersections();
   }

   @Override
   public LineSegment3d getIntersection(int intersectionIndex)
   {
      return intersectionCalculator.getIntersection(intersectionIndex);
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
   
   /** Just for clarity. */
   private static class ConcaveHullVertices extends ArrayList<Point3d>
   {
      private static final long serialVersionUID = -4203566841508179115L;
   }

   /** Just for clarity. */
   private static class ConvexHullVertices extends ArrayList<Point3d>
   {
      private static final long serialVersionUID = -4203566841508179115L;
   }
}
