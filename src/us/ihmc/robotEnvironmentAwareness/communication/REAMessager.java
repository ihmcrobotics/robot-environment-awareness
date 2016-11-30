package us.ihmc.robotEnvironmentAwareness.communication;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

public interface REAMessager
{
   void submitMessage(REAMessage message);

   <T extends Object> AtomicReference<T> createInput(String messageName, T defaultValue);

   default <T extends Object> AtomicReference<T> createInput(String messageName)
   {
      return createInput(messageName, null);
   }
   
   void startMessager() throws IOException;

   void closeMessager();
}