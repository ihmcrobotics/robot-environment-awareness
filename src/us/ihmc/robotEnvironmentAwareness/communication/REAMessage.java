package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.communication.packets.Packet;

public final class REAMessage extends Packet<REAMessage>
{
   public String topic;
   public Object messageContent;

   public REAMessage()
   {
   }

   public REAMessage(String topic, Object messageContent)
   {
      this.topic = topic;
      this.messageContent = messageContent;
   }

   public String getTopic()
   {
      return topic;
   }

   public Object getMessageContent()
   {
      return messageContent;
   }

   @Override
   public boolean epsilonEquals(REAMessage other, double epsilon)
   {
      return topic.equals(other.topic) && messageContent.equals(other.messageContent);
   }
}
