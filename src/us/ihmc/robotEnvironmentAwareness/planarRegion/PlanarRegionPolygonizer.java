package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

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

   public void compute(List<PlanarRegion> planarRegions, PolygonizerParameters parameters)
   {
      for (PlanarRegion planarRegion : planarRegions)
      {
         if (planarRegion.getNumberOfNodes() < parameters.getMinNumberOfNodes())
            continue;
         
         Point3d origin = planarRegion.getOrigin();
         Quat4d orientation = new Quat4d();
         orientation.set(GeometryTools.getRotationBasedOnNormal(planarRegion.getNormal()));
         
         
      }
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
      return toPointInPlane(point, origin, orientation);
   }

   private static List<Point2d> toPointsInPlane(List<Point3d> pointsToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      List<Point2d> pointsInPlane = new ArrayList<>();
      pointsToTransform.parallelStream().map(point -> toPointInPlane(point, planeOrigin, planeOrientation)).forEach(pointsInPlane::add);
      return pointsInPlane;
   }

   private static Point2d toPointInPlane(Point3d pointToTransform, Point3d planeOrigin, Quat4d planeOrientation)
   {
      Point2d pointInPlane = new Point2d();

      double qx = -planeOrientation.getX();
      double qy = -planeOrientation.getY();
      double qz = -planeOrientation.getZ();
      double qs = planeOrientation.getW();

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = pointToTransform.getX() - planeOrigin.getX();
      double y = pointToTransform.getY() - planeOrigin.getY();
      double z = pointToTransform.getZ() - planeOrigin.getZ();

      double crossX = 2.0 * (qy * z - qz * y);
      double crossY = 2.0 * (qz * x - qx * z);
      double crossZ = 2.0 * (qx * y - qy * x);

      double crossCrossX = qy * crossZ - qz * crossY;
      double crossCrossY = qz * crossX - qx * crossZ;

      pointInPlane.setX(x + qs * crossX + crossCrossX);
      pointInPlane.setY(y + qs * crossY + crossCrossY);

      return pointInPlane;
   }

   private Point3d getPointInWorld(Point2d point)
   {
      return toPointInWorld(point, origin, orientation);
   }

   private static List<Point3d> toPointsInWorld(List<Point2d> pointsInPlane, Point3d planeOrigin, Quat4d planeOrientation)
   {
      List<Point3d> pointsInWorld = new ArrayList<>();
      pointsInPlane.parallelStream().map(point -> toPointInWorld(point, planeOrigin, planeOrientation)).forEach(pointsInWorld::add);
      return pointsInWorld;
   }

   private static Point3d toPointInWorld(Point2d pointInPlane, Point3d planeOrigin, Quat4d planeOrientation)
   {
      Point3d pointInWorld = new Point3d();

      double qx = planeOrientation.getX();
      double qy = planeOrientation.getY();
      double qz = planeOrientation.getZ();
      double qs = planeOrientation.getW();

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = pointInPlane.getX();
      double y = pointInPlane.getY();
      double z = 0.0;

      double crossX = 2.0 * (qy * z - qz * y);
      double crossY = 2.0 * (qz * x - qx * z);
      double crossZ = 2.0 * (qx * y - qy * x);

      double crossCrossX = qy * crossZ - qz * crossY;
      double crossCrossY = qz * crossX - qx * crossZ;
      double crossCrossZ = qx * crossY - qy * crossX;

      pointInWorld.setX(x + qs * crossX + crossCrossX);
      pointInWorld.setY(y + qs * crossY + crossCrossY);
      pointInWorld.setZ(z + qs * crossZ + crossCrossZ);
      pointInWorld.add(planeOrigin);

      return pointInWorld;
   }
}
