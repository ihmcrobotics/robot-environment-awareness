package us.ihmc.robotEnvironmentAwareness.communication;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.NetStateListener;
import us.ihmc.tools.io.printing.PrintTools;

public class REAMessagerSharedVariables implements REAMessager
{
   private final AtomicBoolean isConnected = new AtomicBoolean(false);
   private final ConcurrentHashMap<String, List<AtomicReference<Object>>> boundVariables = new ConcurrentHashMap<>();
   private final ConcurrentHashMap<String, List<REATopicListener<Object>>> topicListenersMap = new ConcurrentHashMap<>();
   private final List<NetStateListener> connectionStateListeners = new ArrayList<>();

   public REAMessagerSharedVariables()
   {
   }

   @Override
   public void submitMessage(REAMessage message)
   {
      if (!isConnected.get())
      {
         PrintTools.warn(this, "This messager is closed, message's topic: " + message.getTopic());
         return;
      }

      List<AtomicReference<Object>> boundVariablesForTopic = boundVariables.get(message.getTopic());
      if (boundVariablesForTopic != null)
         boundVariablesForTopic.forEach(variable -> variable.set(message.getMessageContent()));

      List<REATopicListener<Object>> topicListeners = topicListenersMap.get(message.getTopic());
      if (topicListeners != null)
         topicListeners.forEach(listener -> listener.receivedMessageForTopic(message.getMessageContent()));
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
   public <T> void registerTopicListener(String topic, REATopicListener<T> listener)
   {
      List<REATopicListener<Object>> topicListeners = topicListenersMap.get(topic);
      if (topicListeners == null)
      {
         topicListeners = new ArrayList<>();
         topicListenersMap.put(topic, topicListeners);
      }
      topicListeners.add((REATopicListener<Object>) listener);
   }

   @Override
   public void startMessager() throws IOException
   {
      isConnected.set(true);
      connectionStateListeners.forEach(NetStateListener::connected);
   }

   @Override
   public void closeMessager()
   {
      isConnected.set(false);
      connectionStateListeners.forEach(NetStateListener::disconnected);
      boundVariables.clear();
   }

   @Override
   public void registerConnectionStateListener(NetStateListener listener)
   {
      connectionStateListeners.add(listener);
   }
}
