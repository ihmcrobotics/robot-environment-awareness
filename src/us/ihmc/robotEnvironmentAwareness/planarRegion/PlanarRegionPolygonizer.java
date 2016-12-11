package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import javax.vecmath.Point2d;

import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullCollection;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullPruningFilteringTools;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory;
import us.ihmc.robotics.geometry.ConvexPolygon2d;

public abstract class PlanarRegionPolygonizer
{
   public static Map<OcTreeNodePlanarRegion, PlanarRegionConcaveHull> computeConcaveHulls(List<OcTreeNodePlanarRegion> ocTreeNodePlanarRegions, PolygonizerParameters parameters)
   {
      return ocTreeNodePlanarRegions.parallelStream()
                          .filter(region -> region.getNumberOfNodes() >= parameters.getMinNumberOfNodes())
                          .map(region -> createConcaveHull(region, parameters))
                          .collect(Collectors.toConcurrentMap(PlanarRegionConcaveHull::getOcTreeNodePlanarRegion, concaveHull -> concaveHull));
   }

   public static Map<OcTreeNodePlanarRegion, PlanarRegionConvexPolygons> computeConvexDecomposition(Map<OcTreeNodePlanarRegion, PlanarRegionConcaveHull> concaveHulls, PolygonizerParameters parameters)
   {
      return concaveHulls.values().parallelStream()
                  .map(concaveHull -> createConvexPolygons(concaveHull, parameters))
                  .collect(Collectors.toConcurrentMap(PlanarRegionConvexPolygons::getOcTreeNodePlanarRegion, polygons -> polygons));
   }

   private static PlanarRegionConcaveHull createConcaveHull(OcTreeNodePlanarRegion ocTreeNodePlanarRegion, PolygonizerParameters parameters)
   {
      double concaveHullThreshold = parameters.getConcaveHullThreshold();
      double shallowAngleThreshold = parameters.getShallowAngleThreshold();
      double peakAngleThreshold = parameters.getPeakAngleThreshold();
      double lengthThreshold = parameters.getLengthThreshold();

      List<Point2d> pointsInPlane = PolygonizerTools.extractPointsInPlane(ocTreeNodePlanarRegion);
      ConcaveHullCollection concaveHullCollection = SimpleConcaveHullFactory.createConcaveHullCollection(pointsInPlane, concaveHullThreshold);

      for (int i = 0; i < 5; i++)
      {
         ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHullCollection);
         ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHullCollection);
      }

      return new PlanarRegionConcaveHull(ocTreeNodePlanarRegion, concaveHullCollection);
   }

   private static PlanarRegionConvexPolygons createConvexPolygons(PlanarRegionConcaveHull concaveHull, PolygonizerParameters parameters)
   {
      List<ConvexPolygon2d> decomposedPolygons = new ArrayList<>();
      double depthThreshold = parameters.getDepthThreshold();
      ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHull.getConcaveHullCollection(), depthThreshold, decomposedPolygons);

      OcTreeNodePlanarRegion planarRegion = concaveHull.getOcTreeNodePlanarRegion();
      return new PlanarRegionConvexPolygons(planarRegion, decomposedPolygons);
   }
}
