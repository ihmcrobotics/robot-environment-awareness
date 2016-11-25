package us.ihmc.robotEnvironmentAwareness.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;

import java.util.ArrayList;
import java.util.Random;

/**
 * Created by adrien on 11/21/16.
 */
public class OctreeNodeMessage extends Packet<OctreeNodeMessage>
{

   private byte messageID;
   private ArrayList<OctreeNodeData> octreeNodeDataList;
   private float defaultNodeSize;

   public OctreeNodeMessage(Random random)
   {
   }

   public OctreeNodeMessage()
   {
      setDestination(PacketDestination.REA_MODULE); // or use broadcast
   }

   public OctreeNodeMessage(byte messageID, ArrayList<OctreeNodeData> octreeNodeDataList, float defaultNodeSize)
   {
      this.messageID = messageID;
      this.octreeNodeDataList = octreeNodeDataList;
      this.defaultNodeSize = defaultNodeSize;
   }

   public ArrayList<OctreeNodeData> getOctreeNodeDataList()
   {
      return octreeNodeDataList;
   }

   public byte getMessageID()
   {
      return messageID;
   }

   public float getDefaultNodeSize()
   {
      return defaultNodeSize;
   }

   @Override public boolean epsilonEquals(OctreeNodeMessage other, double epsilon)
   {
      return false;
   }
}
