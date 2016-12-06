package us.ihmc.robotEnvironmentAwareness.hardware;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import geometry_msgs.Point;
import geometry_msgs.Quaternion;
import scan_to_cloud.PointCloud2WithSource;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationKryoNetClassLists;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

public class MultisensePointCloud2WithSourceReceiver extends AbstractRosTopicSubscriber<PointCloud2WithSource>
{
   PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.REA_MODULE_PORT, REACommunicationKryoNetClassLists.getPublicNetClassList());

   public MultisensePointCloud2WithSourceReceiver() throws URISyntaxException, IOException
   {
      super(PointCloud2WithSource._TYPE);
      URI masterURI = new URI("http://10.6.192.14:11311");
      RosMainNode rosMainNode = new RosMainNode(masterURI, "LidarScanPublisher", true);
      rosMainNode.attachSubscriber("/singleScanAsCloudWithSource", this);
      rosMainNode.execute();

      packetCommunicator.connect();
   }

   @Override
   public void onNewMessage(PointCloud2WithSource cloudHolder)
   {
      UnpackedPointCloud pointCloudData = RosPointCloudSubscriber.unpackPointsAndIntensities(cloudHolder.getCloud());
      Point3d[] points = pointCloudData.getPoints();

      Point translation = cloudHolder.getTranslation();
      Point3d lidarPosition = new Point3d(translation.getX(), translation.getY(), translation.getZ());
      Quaternion orientation = cloudHolder.getOrientation();
      Quat4d lidarQuaternion = new Quat4d(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getW());

      LidarScanMessage lidarScanMessage = new LidarScanMessage();
      lidarScanMessage.setLidarPosition(lidarPosition);
      lidarScanMessage.setLidarOrientation(lidarQuaternion);
      lidarScanMessage.setScan(points);

      packetCommunicator.send(lidarScanMessage);
   }

   public static void main(String[] args) throws URISyntaxException, IOException
   {
      new MultisensePointCloud2WithSourceReceiver();
   }
}
