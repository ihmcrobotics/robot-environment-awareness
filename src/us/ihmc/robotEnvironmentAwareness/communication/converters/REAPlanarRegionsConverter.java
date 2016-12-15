package us.ihmc.robotEnvironmentAwareness.communication.converters;

import java.util.List;

import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.robotEnvironmentAwareness.communication.packets.LineSegment3dMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OcTreeKeyMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.OcTreeNodePlanarRegion;
import us.ihmc.robotEnvironmentAwareness.updaters.RegionFeaturesProvider;

public class REAPlanarRegionsConverter
{
   public static PlanarRegionSegmentationMessage[] createPlanarRegionSegmentationMessages(RegionFeaturesProvider regionFeaturesProvider)
   {
      return createPlanarRegionSegmentationMessages(regionFeaturesProvider.getOcTreePlanarRegions());
   }

   public static PlanarRegionSegmentationMessage[] createPlanarRegionSegmentationMessages(List<OcTreeNodePlanarRegion> ocTreePlanarRegions)
   {
      PlanarRegionSegmentationMessage[] messages = new PlanarRegionSegmentationMessage[ocTreePlanarRegions.size()];

      for (int regionIndex = 0; regionIndex < ocTreePlanarRegions.size(); regionIndex++)
      {
         OcTreeNodePlanarRegion ocTreeNodePlanarRegion = ocTreePlanarRegions.get(regionIndex);
         messages[regionIndex] = createPlanarRegionSegmentationMessage(ocTreeNodePlanarRegion);
      }
      return messages;
   }

   public static PlanarRegionSegmentationMessage createPlanarRegionSegmentationMessage(OcTreeNodePlanarRegion ocTreeNodePlanarRegion)
   {
      int regionId = ocTreeNodePlanarRegion.getId();
      Point3f origin = new Point3f(ocTreeNodePlanarRegion.getOrigin());
      Vector3f normal = new Vector3f(ocTreeNodePlanarRegion.getNormal());
      OcTreeKeyMessage[] nodeKeys = new OcTreeKeyMessage[ocTreeNodePlanarRegion.getNumberOfNodes()];
      Point3f[] nodeHitLocations = new Point3f[ocTreeNodePlanarRegion.getNumberOfNodes()];

      for (int nodeIndex = 0; nodeIndex < ocTreeNodePlanarRegion.getNumberOfNodes(); nodeIndex++)
      {
         NormalOcTreeNode node = ocTreeNodePlanarRegion.getNode(nodeIndex);
         OcTreeKeyMessage nodeKey = new OcTreeKeyMessage(node.getKeyCopy());
         nodeKeys[nodeIndex] = nodeKey;
         nodeHitLocations[nodeIndex] = new Point3f(node.getHitLocationCopy());
      }
      PlanarRegionSegmentationMessage planarRegionNodeKeysMessage = new PlanarRegionSegmentationMessage(regionId, origin, normal, nodeKeys, nodeHitLocations);
      return planarRegionNodeKeysMessage;
   }

   public static LineSegment3dMessage[] createLineSegment3dMessages(RegionFeaturesProvider regionFeaturesProvider)
   {
      LineSegment3dMessage[] messages = new LineSegment3dMessage[regionFeaturesProvider.getNumberOfPlaneIntersections()];

      for (int i = 0; i < regionFeaturesProvider.getNumberOfPlaneIntersections(); i++)
      {
         messages[i] = new LineSegment3dMessage(regionFeaturesProvider.getIntersection(i));
      }
      return messages;
   }
}
