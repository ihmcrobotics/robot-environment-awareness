package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.jOctoMap.boundingBox.OcTreeSimpleBoundingBox;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OctreeNodeData;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OctreeNodeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.REAMessagePacket;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.updaters.REAOcTreeGraphicsBuilder;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import java.util.ArrayList;

/**
 * Created by adrien on 11/18/16.
 */
public class REACommunicationKryoNetClassList extends NetClassList
{

   public REACommunicationKryoNetClassList()
   {
      registerPacketClass(Packet.class);
      registerPacketField(PacketDestination.class);
      registerPacketClass(REAMessagePacket.class);

      registerPacketField(String.class);

      registerPacketField(byte[].class);
      registerPacketField(char[].class);
      registerPacketField(int[].class);
      registerPacketField(String[].class);

      registerPacketField(Point3d.class);
      registerPacketField(Point3f.class);
      registerPacketField(Quat4d.class);

      registerPacketClass(ArrayList.class);
      registerPacketField(Point3f[].class);

      registerPacketField(REAOcTreeGraphicsBuilder.class);
      registerPacketField(REAOcTreeGraphicsBuilder.ColoringType.class);

      registerPacketField(OcTreeSimpleBoundingBox.class);
      registerPacketField(OcTreeKey.class);

      registerPacketField(NormalEstimationParameters.class);
      registerPacketField(PlanarRegionSegmentationParameters.class);
      registerPacketField(IntersectionEstimationParameters.class);
      registerPacketField(PolygonizerParameters.class);


      registerPacketClass(OctreeNodeMessage.class);
      registerPacketField(OctreeNodeData.class);



   }

}



