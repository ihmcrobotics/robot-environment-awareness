package us.ihmc.robotEnvironmentAwareness.communication;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;

public interface REAMessager
{
   void submitMessage(REAMessage message);

   <T extends Object> AtomicReference<T> createInput(String messageName, T defaultValue);

   default <T extends Object> AtomicReference<T> createInput(String messageName)
   {
      return createInput(messageName, null);
   }

   PacketCommunicator getPacketCommunicator();
}