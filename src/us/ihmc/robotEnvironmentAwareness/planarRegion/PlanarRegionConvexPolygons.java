package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools;
import us.ihmc.robotics.geometry.ConvexPolygon2d;

public class PlanarRegionConvexPolygons
{
   private final OcTreeNodePlanarRegion ocTreeNodePlanarRegion;
   private final List<ConvexPolygon2d> convexPolygonsInPlane;
   private final List<List<Point2d>> convexPolygonsVerticesInPlane = new ArrayList<>();
   private final List<List<Point3d>> convexPolygonsVerticesInWorld;

   public PlanarRegionConvexPolygons(OcTreeNodePlanarRegion ocTreeNodePlanarRegion, List<ConvexPolygon2d> convexPolygonsInPlane)
   {
      this(ocTreeNodePlanarRegion, convexPolygonsInPlane, true);
   }

   public PlanarRegionConvexPolygons(OcTreeNodePlanarRegion ocTreeNodePlanarRegion, List<ConvexPolygon2d> convexPolygonsInPlane, boolean counterClockwiseOrdering)
   {
      this.ocTreeNodePlanarRegion = ocTreeNodePlanarRegion;
      this.convexPolygonsInPlane = convexPolygonsInPlane;
      Point3d planeOrigin = ocTreeNodePlanarRegion.getOrigin();
      Vector3d planeNormal = ocTreeNodePlanarRegion.getNormal();

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
                                   .map(convexPolygonInPlane -> PolygonizerTools.toPointsInWorld(convexPolygonInPlane, planeOrigin, planeNormal))
                                   .collect(Collectors.toList());
   }

   public int getRegionId()
   {
      return ocTreeNodePlanarRegion.getId();
   }

   public OcTreeNodePlanarRegion getOcTreeNodePlanarRegion()
   {
      return ocTreeNodePlanarRegion;
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
