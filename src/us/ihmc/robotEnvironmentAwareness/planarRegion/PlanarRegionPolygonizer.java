package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullPruningFilteringTools;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.GeometryTools;

public class PlanarRegionPolygonizer
{
   private List<Point2d> concaveHullVerticesInPlane = new ArrayList<>();
   private List<Point3d> concaveHullVerticesInWorld = new ArrayList<>();
   private List<List<Point3d>> convexPolygonsVerticesInWorld = new ArrayList<>();

   private final Quat4d orientation = new Quat4d();
   private final Point3d origin = new Point3d();
   private final List<Point2d> pointsInPlane = new ArrayList<>();

   private final PolygonizerParameters parameters = new PolygonizerParameters();

   public PlanarRegionPolygonizer()
   {
   }

   public static Map<PlanarRegion, PlanarRegionConcaveHull> computeConcaveHulls(List<PlanarRegion> planarRegions, PolygonizerParameters parameters)
   {
      return planarRegions.parallelStream()
                          .filter(region -> region.getNumberOfNodes() >= parameters.getMinNumberOfNodes())
                          .map(region -> createConcaveHull(region, parameters))
                          .collect(Collectors.toConcurrentMap(PlanarRegionConcaveHull::getPlanarRegion, concaveHull -> concaveHull));

   }

   public static Map<PlanarRegion, PlanarRegionConvexPolygons> computeConvexDecomposition(Map<PlanarRegion, PlanarRegionConcaveHull> concaveHulls, PolygonizerParameters parameters)
   {
      return concaveHulls.values().parallelStream()
                  .map(concaveHull -> createConvexPolygons(concaveHull, parameters))
                  .collect(Collectors.toConcurrentMap(PlanarRegionConvexPolygons::getPlanarRegion, polygons -> polygons));
   }

   private static PlanarRegionConcaveHull createConcaveHull(PlanarRegion planarRegion, PolygonizerParameters parameters)
   {
      double concaveHullThreshold = parameters.getConcaveHullThreshold();
      double shallowAngleThreshold = parameters.getShallowAngleThreshold();
      double peakAngleThreshold = parameters.getPeakAngleThreshold();
      double lengthThreshold = parameters.getLengthThreshold();

      List<Point2d> pointsInPlane = PolygonizerTools.extractPointsInPlane(planarRegion);
      List<Point2d> concaveHullVerticesInPlane = SimpleConcaveHullFactory.createConcaveHullAsPoint2dList(pointsInPlane, concaveHullThreshold);

      ConcaveHullTools.ensureClockwiseOrdering(concaveHullVerticesInPlane);
      ConcaveHullTools.removeSuccessiveDuplicateVertices(concaveHullVerticesInPlane);

      for (int i = 0; i < 5; i++)
      {
         ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHullVerticesInPlane);
         ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHullVerticesInPlane);
      }

      return new PlanarRegionConcaveHull(planarRegion, concaveHullVerticesInPlane);
   }

   private static PlanarRegionConvexPolygons createConvexPolygons(PlanarRegionConcaveHull concaveHull, PolygonizerParameters parameters)
   {
      List<ConvexPolygon2d> decomposedPolygons = new ArrayList<>();
      double depthThreshold = parameters.getDepthThreshold();
      ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHull.getConcaveHullVerticesInPlane(), depthThreshold, decomposedPolygons);

      PlanarRegion planarRegion = concaveHull.getPlanarRegion();
      return new PlanarRegionConvexPolygons(planarRegion, decomposedPolygons);
   }

   public void compute(PlanarRegion planarRegion)
   {
      concaveHullVerticesInPlane.clear();
      concaveHullVerticesInWorld.clear();
      convexPolygonsVerticesInWorld.clear();

      if (planarRegion.getNumberOfNodes() < parameters.getMinNumberOfNodes())
         return;

      updateRegionProperties(planarRegion);

      concaveHullVerticesInPlane.addAll(SimpleConcaveHullFactory.createConcaveHullAsPoint2dList(pointsInPlane, parameters.getConcaveHullThreshold()));
      
      ConcaveHullTools.ensureClockwiseOrdering(concaveHullVerticesInPlane);
      ConcaveHullTools.removeSuccessiveDuplicateVertices(concaveHullVerticesInPlane);

      for (int i = 0; i < 5; i++)
      {
         double shallowAngleThreshold = parameters.getShallowAngleThreshold();
         double peakAngleThreshold = parameters.getPeakAngleThreshold();
         double lengthThreshold = parameters.getLengthThreshold();
         ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHullVerticesInPlane);
         ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHullVerticesInPlane);
      }

      for (int i = 0; i < concaveHullVerticesInPlane.size(); i++)
      {
         Point3d vertexInWorld = getPointInWorld(concaveHullVerticesInPlane.get(i));
         concaveHullVerticesInWorld.add(vertexInWorld);
      }

      List<ConvexPolygon2d> decomposedPolygons = new ArrayList<>();
      double depthThreshold = parameters.getDepthThreshold();
      ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHullVerticesInPlane, depthThreshold, decomposedPolygons);

      for (int pIndex = 0; pIndex < decomposedPolygons.size(); pIndex++)
      {
         List<Point3d> convexPolygonVerticesInWorld = new ArrayList<>();
         ConvexPolygon2d convexPolygon2d = decomposedPolygons.get(pIndex);

         for (int i = 0; i < convexPolygon2d.getNumberOfVertices(); i++)
         {
            convexPolygonVerticesInWorld.add(getPointInWorld(convexPolygon2d.getVertex(i)));
         }
         Collections.reverse(convexPolygonVerticesInWorld);
         convexPolygonsVerticesInWorld.add(convexPolygonVerticesInWorld);
      }
   }

   public void setParameters(PolygonizerParameters parameters)
   {
      this.parameters.set(parameters);
   }

   public List<Point3d> getConcaveHullVertices()
   {
      return concaveHullVerticesInWorld;
   }

   public int getNumberOfConvexPolygons()
   {
      return convexPolygonsVerticesInWorld.size();
   }

   public List<Point3d> getConvexPolygonVertices(int polygonIndex)
   {
      return convexPolygonsVerticesInWorld.get(polygonIndex);
   }

   private void updateRegionProperties(PlanarRegion planarRegion)
   {
      origin.set(planarRegion.getOrigin());
      orientation.set(GeometryTools.getRotationBasedOnNormal(planarRegion.getNormal()));
      computePointsInPlane(planarRegion);
   }

   private void computePointsInPlane(PlanarRegion planarRegion)
   {
      pointsInPlane.clear();

      Point3d point3d = new Point3d();

      for (int i = 0; i < planarRegion.getNumberOfNodes(); i++)
      {
         planarRegion.getPoint(i, point3d);
         Point2d pointInPlane = getPointInPlane(point3d);
         pointsInPlane.add(pointInPlane);
      }
   }

   private Point2d getPointInPlane(Point3d point)
   {
      return PolygonizerTools.toPointInPlane(point, origin, orientation);
   }

   private Point3d getPointInWorld(Point2d point)
   {
      return PolygonizerTools.toPointInWorld(point, origin, orientation);
   }
}
