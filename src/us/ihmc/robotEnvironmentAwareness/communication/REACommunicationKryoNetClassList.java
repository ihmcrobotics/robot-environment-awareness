package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandCollisionDetectedPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.REAMessagePacket;

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

      registerPacketField(String.class);
      registerPacketField(char[].class);
      registerPacketField(String[].class);
      registerPacketClass(HandCollisionDetectedPacket.class);

      registerPacketField(byte[].class);
      registerPacketField(Point3d.class);
      registerPacketField(Point3f.class);
      registerPacketField(Quat4d.class);
      registerPacketField(PacketDestination.class);


      registerPacketClass(REAMessagePacket.class);
      registerPacketClass(ArrayList.class);
      registerPacketField(Point3f[].class);
   }


}



