package us.ihmc.robotEnvironmentAwareness.hardware;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import sensor_msgs.PointCloud2;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.robotEnvironmentAwareness.communication.LidarPosePacket;
import us.ihmc.robotEnvironmentAwareness.communication.LidarSimulationNetClassList;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

public class MultisensePointCloudReceiver extends RosPointCloudSubscriber
{
   PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.BEHAVIOUR_MODULE_PORT, new LidarSimulationNetClassList());

   public MultisensePointCloudReceiver() throws URISyntaxException, IOException
   {
      URI rosMasterURI = new URI("http://10.6.192.14:11311");
      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "atlas/AtlasMinimalMultisenseMocapNetworkProcessor", true);
//      rosMainNode.attachSubscriber("/multisense/lidar_points2", this);
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
      LidarPosePacket lidarPosePacket = new LidarPosePacket(new Point3d(), new Quat4d());
      packetCommunicator.send(lidarPosePacket);
   }

   public static void main(String[] args) throws URISyntaxException, IOException
   {
      new MultisensePointCloudReceiver();
   }
}
