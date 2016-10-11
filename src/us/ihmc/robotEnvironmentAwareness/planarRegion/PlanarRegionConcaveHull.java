package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.robotics.geometry.GeometryTools;

public class PlanarRegionConcaveHull
{
   private final PlanarRegion planarRegion;
   private final List<Point2d> concaveHullVerticesInPlane;
   private final List<Point3d> concaveHullVerticesInWorld;

   public PlanarRegionConcaveHull(PlanarRegion planarRegion, List<Point2d> concaveHullVerticesInPlane)
   {
      this.planarRegion = planarRegion;
      this.concaveHullVerticesInPlane = concaveHullVerticesInPlane;
      Point3d planeOrigin = planarRegion.getOrigin();
      Quat4d planeOrientation = new Quat4d();
      planeOrientation.set(GeometryTools.getRotationBasedOnNormal(planarRegion.getNormal()));
      this.concaveHullVerticesInWorld = PolygonizerTools.toPointsInWorld(concaveHullVerticesInPlane, planeOrigin, planeOrientation);
   }

   public int getRegionId()
   {
      return planarRegion.getId();
   }

   public PlanarRegion getPlanarRegion()
   {
      return planarRegion;
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
