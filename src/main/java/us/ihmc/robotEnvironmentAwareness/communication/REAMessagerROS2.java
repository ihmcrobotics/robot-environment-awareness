package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

public class REAMessagerROS2 implements REAMessager
{
   private static final boolean DEBUG = false;

   private final APIFactory.API messagerAPI;

   private final ConcurrentHashMap<APIFactory.Topic<?>, List<AtomicReference<Object>>> inputVariablesMap = new ConcurrentHashMap<>();
   private final ConcurrentHashMap<APIFactory.Topic<?>, List<REATopicListener<Object>>> topicListenersMap = new ConcurrentHashMap<>();
   private final List<ConnectionStateListener> connectionStateListeners = new ArrayList<>();

   public static REAMessager createPublishers(APIFactory.API messagerAPI, NetClassList netClassList)
   {

      return new REAMessagerROS2(messagerAPI);
   }

   public static REAMessager createIntraprocess(APIFactory.API messagerAPI, NetClassList netClassList)
   {
      return null; // Unimplemented
   }

   private REAMessagerROS2(APIFactory.API messagerAPI)
   {
      this.messagerAPI = messagerAPI;

   }

   @Override
   public <T> void submitMessage(REAMessage<T> message)
   {

   }

   @Override
   public <T> AtomicReference<T> createInput(APIFactory.Topic<T> topic, T defaultValue)
   {
      return null;
   }

   @Override
   public <T> void registerTopicListener(APIFactory.Topic<T> topic, REATopicListener<T> listener)
   {

   }

   @Override
   public void startMessager() throws IOException
   {

   }

   @Override
   public void closeMessager()
   {

   }

   @Override
   public boolean isMessagerOpen()
   {
      return false;
   }

   @Override
   public void notifyConnectionStateListeners()
   {

   }

   @Override
   public void registerConnectionStateListener(ConnectionStateListener listener)
   {

   }

   @Override
   public APIFactory.API getMessagerAPI()
   {
      return messagerAPI;
   }
}
