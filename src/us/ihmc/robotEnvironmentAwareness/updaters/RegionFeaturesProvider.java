package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.List;

import us.ihmc.robotEnvironmentAwareness.planarRegion.OcTreeNodePlanarRegion;
import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface RegionFeaturesProvider
{
   List<OcTreeNodePlanarRegion> getOcTreePlanarRegions();
   
   PlanarRegionsList getPlanarRegionsList();

   int getNumberOfPlaneIntersections();

   LineSegment3d getIntersection(int index);

}