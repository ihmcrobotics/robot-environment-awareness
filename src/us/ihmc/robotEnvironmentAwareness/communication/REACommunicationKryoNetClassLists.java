package us.ihmc.robotEnvironmentAwareness.communication;

import java.util.ArrayList;

import javax.vecmath.Point2f;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessage;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage.RequestType;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.APIElementId;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoxMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.LineSegment3dMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeNodeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OcTreeKeyMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionNodeKeysMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;

/**
 * Created by adrien on 11/18/16.
 */
public class REACommunicationKryoNetClassLists
{
   private static final NetClassList privateNetClassList = new NetClassList();
   static
   {
      privateNetClassList.registerPacketClass(Packet.class);
      privateNetClassList.registerPacketClass(REAMessage.class);
      privateNetClassList.registerPacketField(PacketDestination.class);
      privateNetClassList.registerPacketField(Boolean.class);
      privateNetClassList.registerPacketField(Double.class);
      privateNetClassList.registerPacketField(Integer.class);
      privateNetClassList.registerPacketField(float[].class);
      privateNetClassList.registerPacketField(int[].class);
      privateNetClassList.registerPacketField(ArrayList.class);
      privateNetClassList.registerPacketField(Point3d.class);
      privateNetClassList.registerPacketField(Point3f.class);
      privateNetClassList.registerPacketField(Point2f.class);
      privateNetClassList.registerPacketField(Vector3f.class);
      privateNetClassList.registerPacketField(Quat4d.class);
      privateNetClassList.registerPacketField(Quat4f.class);
      privateNetClassList.registerPacketField(Point3f[].class);
      privateNetClassList.registerPacketField(Point2f[].class);
      privateNetClassList.registerPacketField(LineSegment3dMessage.class);
      privateNetClassList.registerPacketField(LineSegment3dMessage[].class);
      privateNetClassList.registerPacketField(APIElementId.class);
      privateNetClassList.registerPacketField(LidarScanMessage.class);
      privateNetClassList.registerPacketField(BoxMessage.class);
      privateNetClassList.registerPacketField(BoundingBoxParametersMessage.class);
      privateNetClassList.registerPacketField(NormalEstimationParameters.class);
      privateNetClassList.registerPacketField(PlanarRegionSegmentationParameters.class);
      privateNetClassList.registerPacketField(IntersectionEstimationParameters.class);
      privateNetClassList.registerPacketField(PolygonizerParameters.class);
      privateNetClassList.registerPacketField(NormalOcTreeMessage.class);
      privateNetClassList.registerPacketField(NormalOcTreeNodeMessage.class);
      privateNetClassList.registerPacketField(NormalOcTreeNodeMessage[].class);
      privateNetClassList.registerPacketField(OcTreeKeyMessage.class);
      privateNetClassList.registerPacketField(OcTreeKeyMessage[].class);
      privateNetClassList.registerPacketField(PlanarRegionNodeKeysMessage.class);
      privateNetClassList.registerPacketField(PlanarRegionNodeKeysMessage[].class);
      privateNetClassList.registerPacketField(PlanarRegionsListMessage.class);
      privateNetClassList.registerPacketField(PlanarRegionMessage.class);
   }

   private static final NetClassList publicNetClassList = new NetClassList();

   static
   {
      publicNetClassList.registerPacketClass(Packet.class);
      publicNetClassList.registerPacketClass(LidarScanMessage.class);
      publicNetClassList.registerPacketClass(PlanarRegionsListMessage.class);
      publicNetClassList.registerPacketClass(RequestPlanarRegionsListMessage.class);

      publicNetClassList.registerPacketField(PacketDestination.class);

      publicNetClassList.registerPacketField(ArrayList.class);
      publicNetClassList.registerPacketField(Point3f.class);
      publicNetClassList.registerPacketField(Point2f.class);
      publicNetClassList.registerPacketField(Vector3f.class);
      publicNetClassList.registerPacketField(Point2f[].class);
      publicNetClassList.registerPacketField(RequestType.class);

      publicNetClassList.registerPacketField(PlanarRegionMessage.class);
   }

   public static NetClassList getPublicNetClassList()
   {
      return publicNetClassList;
   }
   
   public static NetClassList getPrivateNetClassList()
   {
      return privateNetClassList;
   }
}
