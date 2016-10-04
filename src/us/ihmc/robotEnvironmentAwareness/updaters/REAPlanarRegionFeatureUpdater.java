package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.octoMap.ocTree.implementations.NormalOcTree;
import us.ihmc.octoMap.planarRegions.PlanarRegion;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionIntersectionCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.time.TimeTools;

public class REAPlanarRegionFeatureUpdater implements RegionFeaturesProvider
{
   private final NormalOcTree octree;

   private final PlanarRegionIntersectionCalculator intersectionCalculator = new PlanarRegionIntersectionCalculator();
   private final PlanarRegionPolygonizer planarRegionPolygonizer = new PlanarRegionPolygonizer();
   
   private int minPlanarRegionSize = 10;

   private final TIntArrayList regionIds = new TIntArrayList();
   private final List<ConcaveHullVertices> concaveHullsVertices = new ArrayList<>();
   private final List<List<ConvexHullVertices>> convexHullsVertices = new ArrayList<>();

   private final AtomicReference<Boolean> enablePolygonizer;
   private final AtomicReference<Boolean> enableIntersectionCalulator;
   private final AtomicReference<IntersectionEstimationParameters> intersectionEstimationParameters;
   private final AtomicReference<PolygonizerParameters> polygonizerParameters;

   public REAPlanarRegionFeatureUpdater(NormalOcTree octree, REAMessageManager inputManager, REAMessager outputMessager)
   {
      this.octree = octree;

      enablePolygonizer = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerEnable);
      enableIntersectionCalulator = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionEnable);
      intersectionEstimationParameters = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionParameters);
      polygonizerParameters = inputManager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerParameters);
   }

   public void update()
   {
      regionIds.reset();
      concaveHullsVertices.clear();
      convexHullsVertices.clear();
      intersectionCalculator.clear();

      if (!isPolygonizerEnabled() && !isIntersectionCalulatorEnabled())
         return;

      long startTime = System.nanoTime();

      updateIntersections();
      updatePolygons();

      long endTime = System.nanoTime();
      System.out.println("Processing concave hulls took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));
   }

   private void updateIntersections()
   {
      if (!isIntersectionCalulatorEnabled())
         return;

      if (intersectionEstimationParameters.get() != null)
         intersectionCalculator.setParameters(intersectionEstimationParameters.getAndSet(null));
      intersectionCalculator.compute(octree.getPlanarRegions());
   }

   private void updatePolygons()
   {
      if (!isPolygonizerEnabled())
         return;

      if (polygonizerParameters.get() != null)
         planarRegionPolygonizer.setParameters(polygonizerParameters.getAndSet(null));

      for (int i = 0; i < octree.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion planarRegion = octree.getPlanarRegion(i);

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

   private boolean isPolygonizerEnabled()
   {
      return enablePolygonizer.get() == null ? false : enablePolygonizer.get();
   }

   private boolean isIntersectionCalulatorEnabled()
   {
      return enableIntersectionCalulator.get() == null ? false : enableIntersectionCalulator.get();
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
