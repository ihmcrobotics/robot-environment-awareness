package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.List;

import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegion;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionConcaveHull;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionConvexPolygons;
import us.ihmc.robotics.geometry.LineSegment3d;

public interface RegionFeaturesProvider
{
   List<PlanarRegion> getPlanarRegions();

   boolean hasPolygonizedPlanarRegions();

   PlanarRegionConcaveHull getPlanarRegionConcaveHull(PlanarRegion planarRegion);

   PlanarRegionConvexPolygons getPlanarRegionConvexPolygons(PlanarRegion planarRegion);

   int getNumberOfPlaneIntersections();

   LineSegment3d getIntersection(int index);

}