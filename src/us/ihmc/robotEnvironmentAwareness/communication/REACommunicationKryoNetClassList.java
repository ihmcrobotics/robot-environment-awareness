package us.ihmc.robotEnvironmentAwareness.communication;

import java.util.ArrayList;

import javax.vecmath.Point2f;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessage;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.jOctoMap.boundingBox.OcTreeSimpleBoundingBox;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeNodeMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.ColoringType;

/**
 * Created by adrien on 11/18/16.
 */
public class REACommunicationKryoNetClassList extends NetClassList
{

   public REACommunicationKryoNetClassList()
   {
      registerPacketClass(Packet.class);
      registerPacketClass(REAMessage.class);

      registerPacketField(PacketDestination.class);

      registerPacketField(String.class);

      registerPacketField(byte[].class);
      registerPacketField(char[].class);
      registerPacketField(int[].class);
      registerPacketField(String[].class);

      registerPacketField(Point3d.class);
      registerPacketField(Vector3f.class);
      registerPacketField(Point3f.class);
      registerPacketField(Quat4d.class);
      registerPacketField(Quat4f.class);

      registerPacketClass(ArrayList.class);
      registerPacketField(Point3f[].class);

      registerPacketField(ColoringType.class);

      registerPacketField(OcTreeSimpleBoundingBox.class);
      registerPacketField(OcTreeKey.class);

      registerPacketField(NormalEstimationParameters.class);
      registerPacketField(PlanarRegionSegmentationParameters.class);
      registerPacketField(IntersectionEstimationParameters.class);
      registerPacketField(PolygonizerParameters.class);

      registerPacketClass(NormalOcTreeMessage.class);
      registerPacketClass(NormalOcTreeNodeMessage.class);
      registerPacketField(NormalOcTreeNodeMessage[].class);
      registerPacketClass(RequestPlanarRegionsListMessage.class);
      registerPacketField(RequestPlanarRegionsListMessage.RequestType.class);

      registerPacketClass(PlanarRegionsListMessage.class);
      registerPacketClass(PlanarRegionMessage.class);
      registerPacketField(Point2f[].class);
      registerPacketField(Point2f.class);
   }
}
