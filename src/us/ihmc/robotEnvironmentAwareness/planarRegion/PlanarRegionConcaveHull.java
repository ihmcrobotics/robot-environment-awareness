package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class PlanarRegionConcaveHull
{
   private final OcTreeNodePlanarRegion ocTreeNodePlanarRegion;
   private final List<Point2d> concaveHullVerticesInPlane;
   private final List<Point3d> concaveHullVerticesInWorld;

   public PlanarRegionConcaveHull(OcTreeNodePlanarRegion planarRegion, List<Point2d> concaveHullVerticesInPlane)
   {
      this.ocTreeNodePlanarRegion = planarRegion;
      this.concaveHullVerticesInPlane = concaveHullVerticesInPlane;
      Point3d planeOrigin = planarRegion.getOrigin();
      Vector3d planeNormal = planarRegion.getNormal();
      this.concaveHullVerticesInWorld = PolygonizerTools.toPointsInWorld(concaveHullVerticesInPlane, planeOrigin, planeNormal);
   }

   public int getRegionId()
   {
      return ocTreeNodePlanarRegion.getId();
   }

   public OcTreeNodePlanarRegion getOcTreeNodePlanarRegion()
   {
      return ocTreeNodePlanarRegion;
   }

   public List<Point2d> getConcaveHullVerticesInPlane()
   {
      return concaveHullVerticesInPlane;
   }

   public List<Point3d> getConcaveHullVerticesInWorld()
   {
      return concaveHullVerticesInWorld;
   }
}
