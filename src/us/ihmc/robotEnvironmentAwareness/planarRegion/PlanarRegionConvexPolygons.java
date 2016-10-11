package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.GeometryTools;

public class PlanarRegionConvexPolygons
{
   private final PlanarRegion planarRegion;
   private final List<ConvexPolygon2d> convexPolygonsInPlane;
   private final List<List<Point2d>> convexPolygonsVerticesInPlane = new ArrayList<>();
   private final List<List<Point3d>> convexPolygonsVerticesInWorld;

   public PlanarRegionConvexPolygons(PlanarRegion planarRegion, List<ConvexPolygon2d> convexPolygonsInPlane)
   {
      this(planarRegion, convexPolygonsInPlane, true);
   }

   public PlanarRegionConvexPolygons(PlanarRegion planarRegion, List<ConvexPolygon2d> convexPolygonsInPlane, boolean counterClockwiseOrdering)
   {
      this.planarRegion = planarRegion;
      this.convexPolygonsInPlane = convexPolygonsInPlane;
      Point3d planeOrigin = planarRegion.getOrigin();
      Quat4d planeOrientation = new Quat4d();
      planeOrientation.set(GeometryTools.getRotationBasedOnNormal(planarRegion.getNormal()));

      for (int pIndex = 0; pIndex < convexPolygonsInPlane.size(); pIndex++)
      {
         ConvexPolygon2d convexPolygon = convexPolygonsInPlane.get(pIndex);
         List<Point2d> convexPolygonVerticesInPlane = new ArrayList<>();
         for (int vIndex = 0; vIndex < convexPolygon.getNumberOfVertices(); vIndex++)
            convexPolygonVerticesInPlane.add(convexPolygon.getVertex(vIndex));

         if (counterClockwiseOrdering)
            ConcaveHullTools.ensureCounterClockwiseOrdering(convexPolygonVerticesInPlane);
         else
            ConcaveHullTools.ensureClockwiseOrdering(convexPolygonVerticesInPlane);

         convexPolygonsVerticesInPlane.add(convexPolygonVerticesInPlane);
      }

      convexPolygonsVerticesInWorld = convexPolygonsVerticesInPlane.parallelStream()
                                   .map(convexPolygonInPlane -> PolygonizerTools.toPointsInWorld(convexPolygonInPlane, planeOrigin, planeOrientation))
                                   .collect(Collectors.toList());
   }

   public int getRegionId()
   {
      return planarRegion.getId();
   }

   public PlanarRegion getPlanarRegion()
   {
      return planarRegion;
   }

   public List<ConvexPolygon2d> getConvexPolygonsInPlane()
   {
      return convexPolygonsInPlane;
   }

   public List<List<Point2d>> getConvexPolygonsVerticesInPlane()
   {
      return convexPolygonsVerticesInPlane;
   }

   public List<List<Point3d>> getConvexPolygonsVerticesInWorld()
   {
      return convexPolygonsVerticesInWorld;
   }
}
