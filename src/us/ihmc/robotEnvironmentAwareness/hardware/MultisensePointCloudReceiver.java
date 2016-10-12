package us.ihmc.robotEnvironmentAwareness.hardware;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import sensor_msgs.PointCloud2;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LidarPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

public class MultisensePointCloudReceiver extends RosPointCloudSubscriber
{
   PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.REA_MODULE_PORT, new IHMCCommunicationKryoNetClassList());

   public MultisensePointCloudReceiver() throws URISyntaxException, IOException
   {
      URI rosMasterURI = new URI("http://10.6.192.14:11311");
//      URI rosMasterURI = new URI("http://10.7.4.100:11311");
      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "atlas/AtlasMinimalMultisenseMocapNetworkProcessor", true);
//      rosMainNode.attachSubscriber("/lidar_to_point_cloud_transformer/assembled_lidar_point_cloud_z_clipped", this);
      rosMainNode.attachSubscriber("/assembled_cloud", this);
      rosMainNode.execute();
      packetCommunicator.connect();

   }

   @Override
   public void onNewMessage(PointCloud2 pointCloud)
   {
      UnpackedPointCloud pointCloudData = unpackPointsAndIntensities(pointCloud);
      Point3d[] points = pointCloudData.getPoints();
      
      PointCloudWorldPacket pointCloudPacket = new PointCloudWorldPacket();
      pointCloudPacket.setDecayingWorldScan(points);
      packetCommunicator.send(pointCloudPacket);
      LidarPosePacket lidarPosePacket = new LidarPosePacket(new Point3d(), new Quat4d(0.0, 0.0, 0.0, 1.0));
      packetCommunicator.send(lidarPosePacket);
   }

   public static void main(String[] args) throws URISyntaxException, IOException
   {
      new MultisensePointCloudReceiver();
   }
}
