package us.ihmc.robotEnvironmentAwareness.communication;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.robotics.lidar.LidarScanParameters;

public class LidarSimulationNetClassList extends NetClassList
{
   public LidarSimulationNetClassList()
   {
      registerPacketClass(LidarPosePacket.class);
      registerPacketClass(PointCloudWorldPacket.class);
      registerPacketClass(DepthDataStateCommand.class);

      registerPacketField(float[].class);
      registerPacketField(Quat4d.class);
      registerPacketField(Point3d.class);
      registerPacketField(LidarScanParameters.class);
   }
}
