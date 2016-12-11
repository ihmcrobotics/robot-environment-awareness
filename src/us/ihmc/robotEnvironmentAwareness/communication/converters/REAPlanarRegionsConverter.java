package us.ihmc.robotEnvironmentAwareness.communication.converters;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import us.ihmc.communication.packets.PlanarRegionMessage;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.robotEnvironmentAwareness.communication.packets.LineSegment3dMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OcTreeKeyMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHull;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullCollection;
import us.ihmc.robotEnvironmentAwareness.planarRegion.OcTreeNodePlanarRegion;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionConcaveHull;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionConvexPolygons;
import us.ihmc.robotEnvironmentAwareness.updaters.RegionFeaturesProvider;
import us.ihmc.robotics.geometry.ConvexPolygon2d;

public class REAPlanarRegionsConverter
{

   public static PlanarRegionMessage createPlanarRegionMessage(PlanarRegionConcaveHull planarRegionConcaveHull, PlanarRegionConvexPolygons planarRegionConvexPolygons)
   {
      OcTreeNodePlanarRegion ocTreeNodePlanarRegion = planarRegionConvexPolygons.getOcTreeNodePlanarRegion();
      Point3f regionOrigin = new Point3f(ocTreeNodePlanarRegion.getOrigin());
      Vector3f regionNormal = new Vector3f(ocTreeNodePlanarRegion.getNormal());

      ConcaveHullCollection concaveHullCollection = planarRegionConcaveHull.getConcaveHullCollection();
      // FIXME update the message so it can carry more than one conave hull
      if (concaveHullCollection.isEmpty())
         return null;

      ConcaveHull concaveHull = concaveHullCollection.iterator().next();

      Point2f[] concaveHullVertices = new Point2f[concaveHull.getNumberOfVertices()];

      for (int vertexIndex = 0; vertexIndex < concaveHull.getNumberOfVertices(); vertexIndex++)
         concaveHullVertices[vertexIndex] = new Point2f(concaveHull.getVertex(vertexIndex));

      List<Point2f[]> convexPolygonsVertices = new ArrayList<>();
      List<ConvexPolygon2d> convexPolygons = planarRegionConvexPolygons.getConvexPolygonsInPlane();

      for (int polygonIndex = 0; polygonIndex < convexPolygons.size(); polygonIndex++)
      {
         ConvexPolygon2d convexPolygon = convexPolygons.get(polygonIndex);
         Point2f[] convexPolygonVertices = new Point2f[convexPolygon.getNumberOfVertices()];
         for (int vertexIndex = 0; vertexIndex < convexPolygon.getNumberOfVertices(); vertexIndex++)
            convexPolygonVertices[vertexIndex] = new Point2f(convexPolygon.getVertex(vertexIndex));
         convexPolygonsVertices.add(convexPolygonVertices);
      }

      PlanarRegionMessage planarRegionMessage = new PlanarRegionMessage(regionOrigin, regionNormal, concaveHullVertices, convexPolygonsVertices);
      planarRegionMessage.setRegionId(planarRegionConvexPolygons.getRegionId());
      return planarRegionMessage;
   }

   public static PlanarRegionsListMessage createPlanarRegionsListMessage(RegionFeaturesProvider regionFeaturesProvider)
   {
      List<OcTreeNodePlanarRegion> ocTreePlanarRegions = regionFeaturesProvider.getOcTreePlanarRegions();
      List<PlanarRegionMessage> planarRegionMessages = new ArrayList<>();

      for (OcTreeNodePlanarRegion ocTreeNodePlanarRegion : ocTreePlanarRegions)
      {
         PlanarRegionConcaveHull planarRegionConcaveHull = regionFeaturesProvider.getPlanarRegionConcaveHull(ocTreeNodePlanarRegion);
         PlanarRegionConvexPolygons planarRegionConvexPolygons = regionFeaturesProvider.getPlanarRegionConvexPolygons(ocTreeNodePlanarRegion);
         if (planarRegionConcaveHull != null && planarRegionConvexPolygons != null)
            planarRegionMessages.add(createPlanarRegionMessage(planarRegionConcaveHull, planarRegionConvexPolygons));
      }

      return new PlanarRegionsListMessage(planarRegionMessages);
   }

   public static PlanarRegionSegmentationMessage[] createPlanarRegionSegmentationMessages(RegionFeaturesProvider regionFeaturesProvider)
   {
      List<OcTreeNodePlanarRegion> ocTreePlanarRegions = regionFeaturesProvider.getOcTreePlanarRegions();
      PlanarRegionSegmentationMessage[] messages = new PlanarRegionSegmentationMessage[ocTreePlanarRegions.size()];

      for (int regionIndex = 0; regionIndex < ocTreePlanarRegions.size(); regionIndex++)
      {
         OcTreeNodePlanarRegion ocTreeNodePlanarRegion = ocTreePlanarRegions.get(regionIndex);
         messages[regionIndex] = createPlanarRegionSegmentationMessage(ocTreeNodePlanarRegion);
      }
      return messages;
   }

   private static PlanarRegionSegmentationMessage createPlanarRegionSegmentationMessage(OcTreeNodePlanarRegion ocTreeNodePlanarRegion)
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
