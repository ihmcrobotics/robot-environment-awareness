package us.ihmc.robotEnvironmentAwareness.communication;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.tools.io.printing.PrintTools;

public class REAMessagerOverNetwork implements REAMessager
{
   private static final boolean DEBUG = false;

   private final ConcurrentHashMap<String, List<AtomicReference<Object>>> inputVariablesMap = new ConcurrentHashMap<>();
   private final ConcurrentHashMap<String, List<REATopicListener<Object>>> topicListenersMap = new ConcurrentHashMap<>();

   private final PacketCommunicator packetCommunicator;

   public static REAMessager createTCPServer(NetworkPorts port, NetClassList netClassList)
   {
      PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(port, netClassList);
      return new REAMessagerOverNetwork(packetCommunicator);
   }

   public static REAMessager createTCPClient(String host, NetworkPorts port, NetClassList netClassList)
   {
      PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(host, port, netClassList);
      return new REAMessagerOverNetwork(packetCommunicator);
   }

   public static REAMessager createIntraprocess(NetworkPorts port, NetClassList netClassList)
   {
      PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(port, netClassList);
      return new REAMessagerOverNetwork(packetCommunicator);
   }

   private REAMessagerOverNetwork(PacketCommunicator packetCommunicator)
   {
      this.packetCommunicator = packetCommunicator;
      this.packetCommunicator.attachListener(REAMessage.class, this::receiveREAMessage);
   }

   private void receiveREAMessage(REAMessage message)
   {
      if (message == null)
         return;

      if (DEBUG)
         PrintTools.info("Packet received from network with message name: " + message.getTopic());

      List<AtomicReference<Object>> inputVariablesForTopic = inputVariablesMap.get(message.getTopic());
      if (inputVariablesForTopic != null)
         inputVariablesForTopic.forEach(variable -> variable.set(message.getMessageContent()));

      List<REATopicListener<Object>> topicListeners = topicListenersMap.get(message.getTopic());
      if (topicListeners != null)
         topicListeners.forEach(listener -> listener.receivedMessageForTopic(message.getMessageContent()));
   }

   @Override
   public void submitMessage(REAMessage message)
   {
      if (!packetCommunicator.isConnected())
      {
         PrintTools.warn(this, "This messager is closed.");
         return;
      }

      if (message.getTopic() == null)
         throw new IllegalArgumentException("Topic is null");

      if (DEBUG)
         PrintTools.info("Submit message for topic: " + message.getTopic());

      // Variable update over network
      packetCommunicator.send(message);
   }

   @Override
   @SuppressWarnings("unchecked")
   public <T> AtomicReference<T> createInput(String topic, T defaultValue)
   {
      AtomicReference<T> boundVariable = new AtomicReference<>(defaultValue);

      List<AtomicReference<Object>> boundVariablesForTopic = inputVariablesMap.get(topic);
      if (boundVariablesForTopic == null)
      {
         boundVariablesForTopic = new ArrayList<>();
         inputVariablesMap.put(topic, boundVariablesForTopic);
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
      packetCommunicator.connect();
   }

   @Override
   public void closeMessager()
   {
      inputVariablesMap.clear();
      packetCommunicator.closeConnection();
      packetCommunicator.close();
   }
}
