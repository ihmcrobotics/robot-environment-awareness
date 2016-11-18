package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;

public interface REAMessager
{
   void submitMessage(REAMessage message);

   List<REAMessage> getUnprocessedMessages();

   @SuppressWarnings("unchecked")
   <T extends Object> AtomicReference<T> createInput(String messageName, T defaultValue);

   default <T extends Object> AtomicReference<T> createInput(String messageName)
   {
      return createInput(messageName, null);
   }

}