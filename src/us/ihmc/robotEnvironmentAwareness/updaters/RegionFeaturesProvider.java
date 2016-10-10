package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.List;

import javax.vecmath.Point3d;

import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegion;
import us.ihmc.robotics.geometry.LineSegment3d;

public interface RegionFeaturesProvider
{
   List<PlanarRegion> getPlanarRegions();

   int getNumberOfConcaveHulls();

   int getRegionId(int concaveHullIndex);

   List<Point3d> getConcaveHull(int concaveHullIndex);

   int getNumberOfConvexHulls(int concaveHullIndex);

   List<Point3d> getConvexHull(int concaveHullIndex, int convexHullIndex);

   int getNumberOfPlaneIntersections();

   LineSegment3d getIntersection(int intersectionIndex);

}