package us.ihmc.robotEnvironmentAwareness.communication;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;

public class REAMessagerSharedVariables implements REAMessager
{
   private final ConcurrentHashMap<String, List<AtomicReference<Object>>> boundVariables = new ConcurrentHashMap<>();

   public REAMessagerSharedVariables()
   {
   }

   @Override
   public void submitMessage(REAMessage message)
   {
      List<AtomicReference<Object>> boundVariablesForTopic = boundVariables.get(message.getMessageName());

      if (boundVariablesForTopic != null)
      {
         for (int i = 0; i < boundVariablesForTopic.size(); i++)
            boundVariablesForTopic.get(i).set(message.getMessageContent());
      }
   }

   @Override
   @SuppressWarnings("unchecked")
   public <T extends Object> AtomicReference<T> createInput(String messageName, T defaultValue)
   {
      AtomicReference<T> boundVariable = new AtomicReference<>(defaultValue);

      List<AtomicReference<Object>> boundVariablesForTopic = boundVariables.get(messageName);
      if (boundVariablesForTopic == null)
      {
         boundVariablesForTopic = new ArrayList<>();
         boundVariables.put(messageName, boundVariablesForTopic);
      }
      boundVariablesForTopic.add((AtomicReference<Object>) boundVariable);
      return boundVariable;
   }

   @Override
   public PacketCommunicator getPacketCommunicator()
   {
      return null;
   }
}
