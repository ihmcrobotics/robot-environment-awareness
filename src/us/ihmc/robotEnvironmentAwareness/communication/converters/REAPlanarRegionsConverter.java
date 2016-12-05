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
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionNodeKeysMessage;
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

      Point2f[] concaveHullVertices = new Point2f[planarRegionConcaveHull.getConcaveHullVerticesInPlane().size()];

      for (int vertexIndex = 0; vertexIndex < planarRegionConcaveHull.getConcaveHullVerticesInPlane().size(); vertexIndex++)
         concaveHullVertices[vertexIndex] = new Point2f(planarRegionConcaveHull.getConcaveHullVerticesInPlane().get(vertexIndex));

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

   public static PlanarRegionNodeKeysMessage[] createPlanarRegionNodeKeysMessages(RegionFeaturesProvider regionFeaturesProvider)
   {
      List<OcTreeNodePlanarRegion> ocTreePlanarRegions = regionFeaturesProvider.getOcTreePlanarRegions();
      PlanarRegionNodeKeysMessage[] messages = new PlanarRegionNodeKeysMessage[ocTreePlanarRegions.size()];

      for (int regionIndex = 0; regionIndex < ocTreePlanarRegions.size(); regionIndex++)
      {
         OcTreeNodePlanarRegion ocTreeNodePlanarRegion = ocTreePlanarRegions.get(regionIndex);
         messages[regionIndex] = createPlanarRegionNodeKeysMessage(ocTreeNodePlanarRegion);
      }
      return messages;
   }

   private static PlanarRegionNodeKeysMessage createPlanarRegionNodeKeysMessage(OcTreeNodePlanarRegion ocTreeNodePlanarRegion)
   {
      int regionId = ocTreeNodePlanarRegion.getId();
      OcTreeKeyMessage[] nodeKeys = new OcTreeKeyMessage[ocTreeNodePlanarRegion.getNumberOfNodes()];

      for (int nodeIndex = 0; nodeIndex < ocTreeNodePlanarRegion.getNumberOfNodes(); nodeIndex++)
      {
         NormalOcTreeNode node = ocTreeNodePlanarRegion.getNode(nodeIndex);
         OcTreeKeyMessage nodeKey = new OcTreeKeyMessage(node.getKeyCopy());
         nodeKeys[nodeIndex] = nodeKey;
      }
      PlanarRegionNodeKeysMessage planarRegionNodeKeysMessage = new PlanarRegionNodeKeysMessage(regionId, nodeKeys);
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
