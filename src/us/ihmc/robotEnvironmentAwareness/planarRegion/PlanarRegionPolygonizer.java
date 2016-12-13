package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import javax.vecmath.Point2d;

import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullCollection;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullPruningFilteringTools;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory;
import us.ihmc.robotics.geometry.ConvexPolygon2d;

public abstract class PlanarRegionPolygonizer
{
   public static Map<OcTreeNodePlanarRegion, PlanarRegionConcaveHull> computeConcaveHulls(List<OcTreeNodePlanarRegion> ocTreeNodePlanarRegions, ConcaveHullFactoryParameters concaveHullFactoryParameters, PolygonizerParameters polygonizerParameters)
   {
      return ocTreeNodePlanarRegions.parallelStream()
                          .filter(region -> region.getNumberOfNodes() >= polygonizerParameters.getMinNumberOfNodes())
                          .map(region -> createConcaveHull(region, concaveHullFactoryParameters, polygonizerParameters))
                          .collect(Collectors.toConcurrentMap(PlanarRegionConcaveHull::getOcTreeNodePlanarRegion, concaveHull -> concaveHull));
   }

   public static Map<OcTreeNodePlanarRegion, PlanarRegionConvexPolygons> computeConvexDecomposition(Map<OcTreeNodePlanarRegion, PlanarRegionConcaveHull> concaveHulls, PolygonizerParameters parameters)
   {
      return concaveHulls.values().parallelStream()
                  .map(concaveHull -> createConvexPolygons(concaveHull, parameters))
                  .collect(Collectors.toConcurrentMap(PlanarRegionConvexPolygons::getOcTreeNodePlanarRegion, polygons -> polygons));
   }

   private static PlanarRegionConcaveHull createConcaveHull(OcTreeNodePlanarRegion ocTreeNodePlanarRegion, ConcaveHullFactoryParameters concaveHullFactoryParameters, PolygonizerParameters polygonizerParameters)
   {
      double shallowAngleThreshold = polygonizerParameters.getShallowAngleThreshold();
      double peakAngleThreshold = polygonizerParameters.getPeakAngleThreshold();
      double lengthThreshold = polygonizerParameters.getLengthThreshold();

      List<Point2d> pointsInPlane = PolygonizerTools.extractPointsInPlane(ocTreeNodePlanarRegion);
      ConcaveHullCollection concaveHullCollection = SimpleConcaveHullFactory.createConcaveHullCollection(pointsInPlane, concaveHullFactoryParameters);

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
