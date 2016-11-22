package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.robotEnvironmentAwareness.communication.packets.REAMessageIdentifier;

public final class REAMessage implements REAMessageIdentifier
{
   private final String messageName;
   private final Object messageContent;

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
}
