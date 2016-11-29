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
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
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
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessageManager;

public class LIDARBasedEnvironmentAwarenessUI extends Application
{
   private static final String SERVER_HOST = NetworkParameters.getHost(NetworkParameterKeys.networkManager);

   private static final String CONFIGURATION_FILE_NAME = "./Configurations/defaultREAConfiguration.txt";

   private final PacketCommunicator packetCommunicator;
   private final RobotEnvironmentAwareness3DScene scene3D = new RobotEnvironmentAwareness3DScene();
   private final BorderPane mainPane;

   private final REAMessageManager uiInputManager = new REAMessageManager();
   private final REAMessageManager uiOutputManager = new REAMessageManager();
   private final LIDARBasedREAModule lidarBasedREAModule = new LIDARBasedREAModule(uiOutputManager, uiInputManager);
   private final REAMeshViewer reaMeshViewer = new REAMeshViewer(uiInputManager);
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

   public LIDARBasedEnvironmentAwarenessUI() throws IOException
   {
      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(SERVER_HOST, NetworkPorts.REA_MODULE_PORT, new IHMCCommunicationKryoNetClassList());

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();

      lidarBasedREAModule.attachListeners(packetCommunicator);
      lidarBasedREAModule.start();
      
      packetCommunicator.attachListener(LidarScanMessage.class, lidarFrameViewer.createLidarScanMessageConsumer());
      lidarFrameViewer.start();

      packetCommunicator.connect();
   }

   @FXML // TODO Move me somewhere else, maybe
   private void sendLidarCommand()
   {
      DepthDataStateCommand packet = new DepthDataStateCommand();
      packet.lidarState = LidarState.ENABLE;
      packet.publishLidarPose = true;
      packetCommunicator.send(packet);
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      mainPane.setCenter(scene3D);

      pointCloudAnchorPaneController.start();
      scene3D.attachChild(pointCloudAnchorPaneController.getRoot());
      scene3D.attachChild(reaMeshViewer.getRoot());
      scene3D.attachChild(lidarFrameViewer.getRoot());

      packetCommunicator.attachListener(PointCloudWorldPacket.class, pointCloudAnchorPaneController.getPointCloudWorldPacketConsumer());
      pointCloudAnchorPaneController.bindControls();

      File configurationFile = new File(CONFIGURATION_FILE_NAME);

      ocTreeBasicsAnchorPaneController.setConfigurationFile(configurationFile);
      ocTreeBasicsAnchorPaneController.attachOutputMessager(uiOutputManager);
      ocTreeBasicsAnchorPaneController.bindControls();

      lidarFilterAnchorPaneController.setConfigurationFile(configurationFile);
      lidarFilterAnchorPaneController.attachOutputMessager(uiOutputManager);
      lidarFilterAnchorPaneController.bindControls();
      
      normalEstimationAnchorPaneController.setConfigurationFile(configurationFile);
      normalEstimationAnchorPaneController.attachOutputMessager(uiOutputManager);
      normalEstimationAnchorPaneController.bindControls();
      
      regionSegmentationAnchorPaneController.setConfigurationFile(configurationFile);
      regionSegmentationAnchorPaneController.attachOutputMessager(uiOutputManager);
      regionSegmentationAnchorPaneController.bindControls();
      
      polygonizerAnchorPaneController.setConfigurationFile(configurationFile);
      polygonizerAnchorPaneController.attachOutputMessager(uiOutputManager);
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
   public void stop()
   {
      try
      {
         packetCommunicator.closeConnection();
         packetCommunicator.close();
         if (scene3D != null)
            scene3D.stop();
         if (pointCloudAnchorPaneController != null)
            pointCloudAnchorPaneController.stop();
         lidarBasedREAModule.stop();
         reaMeshViewer.stop();
         lidarFrameViewer.stop();
         Platform.exit();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
