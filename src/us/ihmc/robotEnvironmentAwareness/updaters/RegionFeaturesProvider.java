package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.List;

import us.ihmc.robotEnvironmentAwareness.planarRegion.OcTreeNodePlanarRegion;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionConcaveHull;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionConvexPolygons;
import us.ihmc.robotics.geometry.LineSegment3d;

public interface RegionFeaturesProvider
{
   List<OcTreeNodePlanarRegion> getOcTreePlanarRegions();

   boolean hasPolygonizedOcTreeNodePlanarRegions();

   PlanarRegionConcaveHull getPlanarRegionConcaveHull(OcTreeNodePlanarRegion ocTreeNodePlanarRegion);

   PlanarRegionConvexPolygons getPlanarRegionConvexPolygons(OcTreeNodePlanarRegion ocTreeNodePlanarRegion);

   int getNumberOfPlaneIntersections();

   LineSegment3d getIntersection(int index);

}