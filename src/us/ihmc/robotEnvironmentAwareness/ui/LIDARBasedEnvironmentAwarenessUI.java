package us.ihmc.robotEnvironmentAwareness.ui;

import java.io.IOException;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.SplitPane;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.robotEnvironmentAwareness.communication.LidarSimulationNetClassList;
import us.ihmc.robotEnvironmentAwareness.ui.controller.LIDARFilterAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.NormalEstimationAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.OcTreeBasicsAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.PointCloudAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.RegionSegmentationAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.scene3D.RobotEnvironmentAwareness3DScene;

public class LIDARBasedEnvironmentAwarenessUI extends Application
{
   private final PacketCommunicator packetCommunicator;
   private final RobotEnvironmentAwareness3DScene scene3D = new RobotEnvironmentAwareness3DScene();
   private final SplitPane mainPane;

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

   public LIDARBasedEnvironmentAwarenessUI() throws IOException
   {
      LidarSimulationNetClassList netClassList = new LidarSimulationNetClassList();
      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient("localhost", NetworkPorts.BEHAVIOUR_MODULE_PORT, netClassList);
      packetCommunicator.connect();

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      mainPane.getItems().set(0, scene3D);

      pointCloudAnchorPaneController.start();
      scene3D.attachChild(pointCloudAnchorPaneController.getRoot());
      packetCommunicator.attachListener(PointCloudWorldPacket.class, pointCloudAnchorPaneController.getPointCloudWorldPacketConsumer());
      pointCloudAnchorPaneController.bindControls();

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);
      primaryStage.setScene(mainScene);
      primaryStage.show();
      // Get the divider to move back we want it to be... Kinda lame.
      Platform.runLater(() -> {
         mainPane.setDividerPositions(0.7);
      });

      primaryStage.setOnCloseRequest(new EventHandler<WindowEvent>()
      {
         @Override
         public void handle(WindowEvent event)
         {
            stop();
         }
      });

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
