package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.communication.packets.Packet;

public final class REAMessage extends Packet<REAMessage>
{
   public String messageName;
   public Object messageContent;

   public REAMessage()
   {
   }

   public REAMessage(String messageName, Object messageContent)
   {
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

   @Override
   public boolean epsilonEquals(REAMessage other, double epsilon)
   {
      return messageName.equals(other.messageName) && messageContent.equals(other.messageContent);
   }
}
