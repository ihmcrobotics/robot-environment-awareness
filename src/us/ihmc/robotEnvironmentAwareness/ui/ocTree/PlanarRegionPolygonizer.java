package us.ihmc.robotEnvironmentAwareness.ui.ocTree;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.opensphere.geometry.algorithm.ConcaveHull;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.MultiPoint;

import us.ihmc.octoMap.planarRegions.PlanarRegion;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullPruningFilteringTools;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.GeometryTools;

public class PlanarRegionPolygonizer
{
   private double concaveHullThreshold = 0.05;
   private List<Point2d> concaveHullVerticesInPlane = new ArrayList<>();
   private List<Point3d> concaveHullVerticesInWorld = new ArrayList<>();
   private List<List<Point3d>> convexPolygonsVerticesInWorld = new ArrayList<>();

   private final Quat4d orientation = new Quat4d();
   private final Point3d origin = new Point3d();
   private final List<Point2d> pointsInPlane = new ArrayList<>();

   private int minNumberOfNodes = 10;

   public PlanarRegionPolygonizer()
   {
   }

   public void compute(PlanarRegion planarRegion)
   {
      concaveHullVerticesInPlane.clear();
      concaveHullVerticesInWorld.clear();
      convexPolygonsVerticesInWorld.clear();

      if (planarRegion.getNumberOfNodes() < minNumberOfNodes)
         return;

      updateRegionProperties(planarRegion);

      Coordinate[] coordinates = new Coordinate[planarRegion.getNumberOfNodes()];
      for (int i = 0; i < planarRegion.getNumberOfNodes(); i++)
      {
         Point2d pointInPlane = pointsInPlane.get(i);
         coordinates[i] = new Coordinate(pointInPlane.getX(), pointInPlane.getY());
      }
      GeometryFactory geometryFactory = new GeometryFactory();
      MultiPoint multiPoint = geometryFactory.createMultiPoint(coordinates);

      Geometry concaveHullGeometry = new ConcaveHull(multiPoint, concaveHullThreshold).getConcaveHull();

      for (Coordinate vertex : concaveHullGeometry.getCoordinates())
      {
         Point2d vertexInPlane = new Point2d(vertex.x, vertex.y);
         concaveHullVerticesInPlane.add(vertexInPlane);
      }

      ConcaveHullTools.ensureClockwiseOrdering(concaveHullVerticesInPlane);

      double shallowAngleThreshold = Math.toRadians(1.0);
      double peakAngleThreshold = Math.toRadians(120.0);
      double percentageThreshold = 0.995;
      double depthThreshold = 0.10;
      double lengthThreshold = 0.01; //sd / 10.0;

      for (int i = 0; i < 5; i++)
      {
         ConcaveHullPruningFilteringTools.filterOutGroupsOfShallowVertices(percentageThreshold, concaveHullVerticesInPlane);
         ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHullVerticesInPlane);
         ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHullVerticesInPlane);
      }

      for (int i = 0; i < concaveHullVerticesInPlane.size(); i++)
      {
         Point3d vertexInWorld = getPointInWorld(concaveHullVerticesInPlane.get(i));
         concaveHullVerticesInWorld.add(vertexInWorld);
      }

      List<ConvexPolygon2d> decomposedPolygons = new ArrayList<>();
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

      for (int i = 0; i < planarRegion.getNumberOfNodes(); i++)
      {
         Point3d point = planarRegion.getPoint(i);
         Point2d pointInPlane = getPointInPlane(point);
         pointsInPlane.add(pointInPlane);
      }
   }

   private Point2d getPointInPlane(Point3d point)
   {
      Point2d pointInPlane = new Point2d();

      double qx = -orientation.getX();
      double qy = -orientation.getY();
      double qz = -orientation.getZ();
      double qs = orientation.getW();

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = point.getX() - origin.getX();
      double y = point.getY() - origin.getY();
      double z = point.getZ() - origin.getZ();

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
      Point3d pointInWorld = new Point3d();

      double qx = orientation.getX();
      double qy = orientation.getY();
      double qz = orientation.getZ();
      double qs = orientation.getW();

      // t = 2.0 * cross(q.xyz, v);
      // v' = v + q.s * t + cross(q.xyz, t);
      double x = point.getX();
      double y = point.getY();
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
      pointInWorld.add(origin);

      return pointInWorld;
   }
}
