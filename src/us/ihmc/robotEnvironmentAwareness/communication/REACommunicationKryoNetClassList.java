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
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeNodeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionNodeKeysMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;

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

      registerPacketField(ArrayList.class);
      registerPacketField(Point3f[].class);

      registerPacketField(BoundingBoxParametersMessage.class);
      registerPacketField(NormalEstimationParameters.class);
      registerPacketField(PlanarRegionSegmentationParameters.class);
      registerPacketField(IntersectionEstimationParameters.class);
      registerPacketField(PolygonizerParameters.class);

      registerPacketField(NormalOcTreeMessage.class);
      registerPacketField(NormalOcTreeNodeMessage.class);
      registerPacketField(NormalOcTreeNodeMessage[].class);

      registerPacketField(PlanarRegionNodeKeysMessage.class);
      registerPacketField(PlanarRegionNodeKeysMessage[].class);

      registerPacketField(PlanarRegionsListMessage.class);
      registerPacketField(PlanarRegionMessage.class);
      registerPacketField(Point2f[].class);
      registerPacketField(Point2f.class);
   }
}
