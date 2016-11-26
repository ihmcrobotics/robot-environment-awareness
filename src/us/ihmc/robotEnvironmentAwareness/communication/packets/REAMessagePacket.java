package us.ihmc.robotEnvironmentAwareness.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;

import java.util.Random;

/**
 * Created by adrien on 11/16/16.
 */
public class REAMessagePacket extends Packet<REAMessagePacket> implements REAMessageIdentifier
{

   public String messageName;
   public Object messageContent; // TODO try to narrow down the type of the object

   public REAMessagePacket(Random random)
   {

   }

   public REAMessagePacket()
   {
      setDestination(PacketDestination.REA_MODULE); // or use broadcast
   }

   public REAMessagePacket(String messageName, Object messageContent)
   {
      setDestination(PacketDestination.REA_MODULE); // or use broadcast
      this.messageName = messageName;
      this.messageContent = messageContent;
   }

   public String getMessageName()
   {
      return messageName;
   }

   public Object getMessageContent()
   {
      return messageContent;
   }

   @Override public boolean epsilonEquals(REAMessagePacket other, double epsilon)
   {
      return false;
   }
}
