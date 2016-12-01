package us.ihmc.robotEnvironmentAwareness.communication;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

public class REAMessagerSharedVariables implements REAMessager
{
   private final ConcurrentHashMap<String, List<AtomicReference<Object>>> boundVariables = new ConcurrentHashMap<>();
   private final ConcurrentHashMap<String, List<REAMessageListener<Object>>> listeners = new ConcurrentHashMap<>();

   public REAMessagerSharedVariables()
   {
   }

   @Override
   public void submitMessage(REAMessage message)
   {
      List<AtomicReference<Object>> boundVariablesForTopic = boundVariables.get(message.getTopic());
      if (boundVariablesForTopic != null)
         boundVariablesForTopic.forEach(variable -> variable.set(message.getMessageContent()));

      List<REAMessageListener<Object>> topicListeners = listeners.get(message.getTopic());
      if (topicListeners != null)
         topicListeners.forEach(listener -> listener.receivedREAMessage(message.getMessageContent()));
   }

   @Override
   @SuppressWarnings("unchecked")
   public <T> AtomicReference<T> createInput(String topic, T defaultValue)
   {
      AtomicReference<T> boundVariable = new AtomicReference<>(defaultValue);

      List<AtomicReference<Object>> boundVariablesForTopic = boundVariables.get(topic);
      if (boundVariablesForTopic == null)
      {
         boundVariablesForTopic = new ArrayList<>();
         boundVariables.put(topic, boundVariablesForTopic);
      }
      boundVariablesForTopic.add((AtomicReference<Object>) boundVariable);
      return boundVariable;
   }

   @Override
   @SuppressWarnings("unchecked")
   public <T> void registerListener(String topic, REAMessageListener<T> listener)
   {
      List<REAMessageListener<Object>> topicListeners = listeners.get(topic);
      if (topicListeners == null)
      {
         topicListeners = new ArrayList<>();
         listeners.put(topic, topicListeners);
      }
      topicListeners.add((REAMessageListener<Object>) listener);
   }

   @Override
   public void startMessager() throws IOException
   {
   }

   @Override
   public void closeMessager()
   {
      boundVariables.clear();
   }
}
