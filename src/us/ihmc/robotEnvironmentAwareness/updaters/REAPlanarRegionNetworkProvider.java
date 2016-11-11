package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessage;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.OcTreeNodePlanarRegion;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionConvexPolygons;
import us.ihmc.robotics.geometry.ConvexPolygon2d;

public class REAPlanarRegionNetworkProvider
{
   private final Set<PacketDestination> listenersForContinuousUpdate = new HashSet<>();
   private final Set<PacketDestination> listenersForSingleUpdate = new HashSet<>();
   private PacketCommunicator packetCommunicator;
   private final RegionFeaturesProvider regionFeaturesProvider;

   public REAPlanarRegionNetworkProvider(RegionFeaturesProvider regionFeaturesProvider)
   {
      this.regionFeaturesProvider = regionFeaturesProvider;
   }

   public void attachPacketCommunicator(PacketCommunicator packetCommunicator)
   {
      this.packetCommunicator = packetCommunicator;
      packetCommunicator.attachListener(RequestPlanarRegionsListMessage.class, this::handlePacket);
   }
   
   public void update(boolean planarRegionsHaveBeenUpdated)
   {
      if (packetCommunicator == null)
         return;

      processRequests();

      boolean hasAtLeastOneListener = !listenersForContinuousUpdate.isEmpty() || !listenersForSingleUpdate.isEmpty();

      if (!hasAtLeastOneListener)
         return;

      // By doing so, this module does not flood the network with useless data.
      if (planarRegionsHaveBeenUpdated)
      {
         for (PacketDestination packetDestination : listenersForContinuousUpdate)
         {
            PlanarRegionsListMessage planarRegionsListMessage = createPlanarRegionsListMessage();
            planarRegionsListMessage.setDestination(packetDestination);
            packetCommunicator.send(planarRegionsListMessage);
         }
      }

      for (PacketDestination packetDestination : listenersForSingleUpdate)
      {
         PlanarRegionsListMessage planarRegionsListMessage = createPlanarRegionsListMessage();
         planarRegionsListMessage.setDestination(packetDestination);
         packetCommunicator.send(planarRegionsListMessage);
      }

      listenersForSingleUpdate.clear();
   }

   private PlanarRegionsListMessage createPlanarRegionsListMessage()
   {
      List<OcTreeNodePlanarRegion> ocTreePlanarRegions = regionFeaturesProvider.getOcTreePlanarRegions();
      List<PlanarRegionMessage> planarRegionMessages = new ArrayList<>();

      for (OcTreeNodePlanarRegion ocTreeNodePlanarRegion : ocTreePlanarRegions)
      {
         PlanarRegionConvexPolygons planarRegionConvexPolygons = regionFeaturesProvider.getPlanarRegionConvexPolygons(ocTreeNodePlanarRegion);
         planarRegionMessages.add(createPlanarRegionMessage(planarRegionConvexPolygons));
      }

      return new PlanarRegionsListMessage(planarRegionMessages);
   }

   private PlanarRegionMessage createPlanarRegionMessage(PlanarRegionConvexPolygons planarRegionConvexPolygons)
   {
      OcTreeNodePlanarRegion ocTreeNodePlanarRegion = planarRegionConvexPolygons.getOcTreeNodePlanarRegion();
      Point3f regionOrigin = new Point3f(ocTreeNodePlanarRegion.getOrigin());
      Vector3f regionNormal = new Vector3f(ocTreeNodePlanarRegion.getNormal());
      List<Point2f[]> convexPolygonsVertices = new ArrayList<>();

      List<ConvexPolygon2d> convexPolygons = planarRegionConvexPolygons.getConvexPolygonsInPlane();

      for (int polygonIndex = 0; polygonIndex < convexPolygons.size(); polygonIndex++)
      {
         ConvexPolygon2d convexPolygon = convexPolygons.get(polygonIndex);
         Point2f[] convexPolygonVertices = new Point2f[convexPolygon.getNumberOfVertices()];
         for (int vertexIndex = 0; vertexIndex < convexPolygon.getNumberOfVertices(); vertexIndex++)
            convexPolygonVertices[vertexIndex] = new Point2f(convexPolygon.getVertex(vertexIndex));
      }

      return new PlanarRegionMessage(regionOrigin, regionNormal, convexPolygonsVertices);
   }

   private void processRequests()
   {
      while (!requestsToProcess.isEmpty())
      {
         RequestPlanarRegionsListMessage request = requestsToProcess.poll();
         PacketDestination source = PacketDestination.fromOrdinal(request.getSource());
         switch (request.getRequesType())
         {
         case CONTINUOUS_UPDATE:
            listenersForContinuousUpdate.add(source);
            break;
         case SINGLE_UPDATE:
            listenersForSingleUpdate.add(source);
            break;
         case STOP_UPDATE:
            listenersForContinuousUpdate.remove(source);
            break;
         default:
            break;
         }
      }
   }

   private final ConcurrentLinkedQueue<RequestPlanarRegionsListMessage> requestsToProcess = new ConcurrentLinkedQueue<>();

   private void handlePacket(RequestPlanarRegionsListMessage packet)
   {
      if (packet != null)
         requestsToProcess.offer(packet);
   }
}
