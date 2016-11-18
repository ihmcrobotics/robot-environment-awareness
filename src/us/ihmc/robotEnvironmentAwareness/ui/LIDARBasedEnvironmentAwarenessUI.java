package us.ihmc.robotEnvironmentAwareness.ui;

import java.io.File;
import java.io.IOException;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LidarPosePacket;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotEnvironmentAwareness.ui.controller.LIDARFilterAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.NormalEstimationAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.OcTreeBasicsAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.PointCloudAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.PolygonizerAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.RegionSegmentationAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.scene3D.RobotEnvironmentAwareness3DScene;
import us.ihmc.robotEnvironmentAwareness.ui.viewer.LidarFrameViewer;
import us.ihmc.robotEnvironmentAwareness.ui.viewer.REAMeshViewer;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessagerOverNetwork;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessagerSharedVariables;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessager;

public class LIDARBasedEnvironmentAwarenessUI extends Application {

    private static final String SERVER_HOST = "localhost";
    private static final String CONFIGURATION_FILE_NAME = "./Configurations/defaultREAConfiguration.txt";


    private final PacketCommunicator reaModulePacketCommunicator;
    private final RobotEnvironmentAwareness3DScene scene3D = new RobotEnvironmentAwareness3DScene();
    private final BorderPane mainPane;

    private final REAMessager reaMessager;

    private final REAMeshViewer reaMeshViewer;
    private final LidarFrameViewer lidarFrameViewer = new LidarFrameViewer();


    @FXML
    private PointCloudAnchorPaneController pointCloudAnchorPaneController;
    @FXML
    private OcTreeBasicsAnchorPaneController ocTreeBasicsAnchorPaneController;
    @FXML
    private LIDARFilterAnchorPaneController lidarFilterAnchorPaneController;
    @FXML
    private NormalEstimationAnchorPaneController normalEstimationAnchorPaneController;
    @FXML
    private RegionSegmentationAnchorPaneController regionSegmentationAnchorPaneController;
    @FXML
    private PolygonizerAnchorPaneController polygonizerAnchorPaneController;

    public LIDARBasedEnvironmentAwarenessUI() throws IOException {

        reaModulePacketCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.REA_MODULE_UI_PORT, new IHMCCommunicationKryoNetClassList());
        reaModulePacketCommunicator.connect();
        reaMessager = new REAMessagerOverNetwork(reaModulePacketCommunicator);

        reaMeshViewer = new REAMeshViewer(reaMessager);

        FXMLLoader loader = new FXMLLoader();
        loader.setController(this);
        loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

        mainPane = loader.load();

        lidarFrameViewer.start();

        // TODO get LIDAR pose from the service
        reaModulePacketCommunicator.attachListener(LidarPosePacket.class, lidarFrameViewer.createLidarPosePacketConsumer());
    }

//   @FXML // TODO Move me somewhere else, maybe
//   private void sendLidarCommand()
//   {
//      DepthDataStateCommand packet = new DepthDataStateCommand();
//      packet.lidarState = LidarState.ENABLE;
//      packet.publishLidarPose = true;
//      packetCommunicator.send(packet);
//   }

    @Override
    public void start(Stage primaryStage) throws Exception {
        mainPane.setCenter(scene3D);

        pointCloudAnchorPaneController.start();
        scene3D.attachChild(pointCloudAnchorPaneController.getRoot());
        scene3D.attachChild(reaMeshViewer.getRoot());
        scene3D.attachChild(lidarFrameViewer.getRoot());

        pointCloudAnchorPaneController.bindControls();

        File configurationFile = new File(CONFIGURATION_FILE_NAME);

        ocTreeBasicsAnchorPaneController.setConfigurationFile(configurationFile);
        ocTreeBasicsAnchorPaneController.attachREAMessager(reaMessager);
        ocTreeBasicsAnchorPaneController.bindControls();

        lidarFilterAnchorPaneController.setConfigurationFile(configurationFile);
        lidarFilterAnchorPaneController.attachREAMessager(reaMessager);
        lidarFilterAnchorPaneController.bindControls();

        normalEstimationAnchorPaneController.setConfigurationFile(configurationFile);
        normalEstimationAnchorPaneController.attachREAMessager(reaMessager);
        normalEstimationAnchorPaneController.bindControls();

        regionSegmentationAnchorPaneController.setConfigurationFile(configurationFile);
        regionSegmentationAnchorPaneController.attachREAMessager(reaMessager);
        regionSegmentationAnchorPaneController.bindControls();

        polygonizerAnchorPaneController.setConfigurationFile(configurationFile);
        polygonizerAnchorPaneController.attachREAMessager(reaMessager);
        polygonizerAnchorPaneController.bindControls();

        reaMeshViewer.start();

        primaryStage.setTitle(getClass().getSimpleName());
        primaryStage.setMaximized(true);
        Scene mainScene = new Scene(mainPane, 600, 400);
        primaryStage.setScene(mainScene);
        primaryStage.show();
        primaryStage.setOnCloseRequest(event -> stop());

    }

    @Override
    public void stop() {
        try {
            reaModulePacketCommunicator.closeConnection();
            reaModulePacketCommunicator.close();

            if (scene3D != null)
                scene3D.stop();
            if (pointCloudAnchorPaneController != null)
                pointCloudAnchorPaneController.stop();

            reaMeshViewer.stop();
            lidarFrameViewer.stop();
            Platform.exit();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static void main(String[] args) {
        launch(args);
    }
}
