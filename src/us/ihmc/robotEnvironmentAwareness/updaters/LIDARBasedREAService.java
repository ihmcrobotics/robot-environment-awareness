package us.ihmc.robotEnvironmentAwareness.updaters;

import javafx.application.Platform;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.stage.Stage;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LidarPosePacket;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;

import java.io.IOException;

/**
 * Created by adrien on 11/16/16.
 */
public class LIDARBasedREAService {

    private static final String SERVER_HOST = "localhost";

    private final REAMessager reaMessager;
    private final LIDARBasedREAModule lidarBasedREAModule;

    private final PacketCommunicator packetCommunicator;
    private final PacketCommunicator reaUIPacketCommunicator;


    public LIDARBasedREAService() throws IOException
    {
        //      Packet communicator receives UI data
        reaUIPacketCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(SERVER_HOST, NetworkPorts.REA_MODULE_UI_PORT, new IHMCCommunicationKryoNetClassList());
        reaMessager = new REAMessagerOverNetwork(reaUIPacketCommunicator);
        reaUIPacketCommunicator.connect();


        System.out.println("Packet communicator UI");

        // Packet communicator client receives Lidar data
        packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(SERVER_HOST, NetworkPorts.REA_MODULE_PORT, new IHMCCommunicationKryoNetClassList());
        lidarBasedREAModule = new LIDARBasedREAModule(reaMessager);
        lidarBasedREAModule.attachListeners(packetCommunicator);
        packetCommunicator.connect();

        System.out.println("Packet communicator Lidar Data");
    }


    public void start() throws Exception
    {
        lidarBasedREAModule.start();
    }

    public void stop()
    {
        try
        {
            packetCommunicator.closeConnection();
            packetCommunicator.close();
            lidarBasedREAModule.stop();
            Platform.exit();
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }


    public static void main(String[] args) throws Exception {

        LIDARBasedREAService lidarBasedREAService = new LIDARBasedREAService();
        lidarBasedREAService.start();

        System.out.println("Main Started");
    }
}
