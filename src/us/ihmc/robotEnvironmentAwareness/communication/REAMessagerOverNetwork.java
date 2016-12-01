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
      this.packetCommunicator.attachListener(REAMessagePacket.class, this::receiveREAMessagePacket);
   }

   private void receiveREAMessagePacket(REAMessagePacket packet)
   {
      if (packet == null)
         return;

      if (DEBUG)
         PrintTools.info("Packet received from network with message name: " + packet.getMessageName());

      List<AtomicReference<Object>> boundVariablesForTopic = inputVariablesMap.get(packet.getMessageName());
      if (boundVariablesForTopic != null)
      {
         for (int i = 0; i < boundVariablesForTopic.size(); i++)
            boundVariablesForTopic.get(i).set(packet.getMessageContent());
      }
   }

   @Override
   public void submitMessage(REAMessage message)
   {
      if (!packetCommunicator.isConnected())
      {
         PrintTools.warn(this, "This messager is closed.");
         return;
      }

      if (message.getMessageName() == null)
         throw new IllegalArgumentException("message name is null");

      if (DEBUG)
         PrintTools.info("Submit message: " + message.getMessageName());

      // Variable update over network
      packetCommunicator.send(new REAMessagePacket(message.getMessageName(), message.getMessageContent()));

      // Local update of variable map
      List<AtomicReference<Object>> boundVariablesForTopic = inputVariablesMap.get(message.getMessageName());
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

      List<AtomicReference<Object>> boundVariablesForTopic = inputVariablesMap.get(messageName);
      if (boundVariablesForTopic == null)
      {
         boundVariablesForTopic = new ArrayList<>();
         inputVariablesMap.put(messageName, boundVariablesForTopic);
      }
      boundVariablesForTopic.add((AtomicReference<Object>) boundVariable);
      return boundVariable;
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
