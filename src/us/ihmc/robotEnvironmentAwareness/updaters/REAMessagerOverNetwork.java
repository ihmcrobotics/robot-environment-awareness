package us.ihmc.robotEnvironmentAwareness.updaters;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.sensing.REAMessagePacket;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

public class REAMessagerOverNetwork implements REAMessager {

    private final ConcurrentHashMap<String, List<AtomicReference<Object>>> inputVariablesMap = new ConcurrentHashMap<>();
    private final PacketCommunicator packetCommunicator;


    public REAMessagerOverNetwork(PacketCommunicator packetCommunicator)
    {
        this.packetCommunicator = packetCommunicator;
        this.packetCommunicator.attachListener(REAMessagePacket.class, reaMessagePacketConsumer);
    }


    public PacketConsumer<REAMessagePacket> reaMessagePacketConsumer = (PacketConsumer<REAMessagePacket>) packet ->
    {
        if (packet == null)
            return;

        System.out.println("Packet received with message: "+ packet.getMessageName());

        List<AtomicReference<Object>> boundVariablesForTopic = inputVariablesMap.get(packet.getMessageName());
        if (boundVariablesForTopic != null)
        {
            for (int i = 0; i < boundVariablesForTopic.size(); i++)
                boundVariablesForTopic.get(i).set(packet.getMessageContent());
        }
    };


    @Override
    public void submitMessage(REAMessage message)
    {
        if (message.getMessageName() == null)
            throw new IllegalArgumentException("message name is null");

        System.out.println("Submit message: "+ message.getMessageName());

        packetCommunicator.send(new REAMessagePacket(message.getMessageName(), message.getMessageContent()));
    }


    @SuppressWarnings("unchecked")
    public <T extends Object> AtomicReference<T> createInput(String messageName, T defaultValue)
    {
        AtomicReference<T> boundVariable = new AtomicReference<T>(defaultValue);

        List<AtomicReference<Object>> boundVariablesForTopic = inputVariablesMap.get(messageName);
        if (boundVariablesForTopic == null) {
            boundVariablesForTopic = new ArrayList<>();
            inputVariablesMap.put(messageName, boundVariablesForTopic);
        }
        boundVariablesForTopic.add((AtomicReference<Object>) boundVariable);
        return boundVariable;
    }

    @Override
    public List<REAMessage> getUnprocessedMessages() {
        return null;
    }
}
