package us.ihmc.robotEnvironmentAwareness.ui;

import java.io.File;
import java.io.IOException;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationKryoNetClassList;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessagerOverNetwork;
import us.ihmc.robotEnvironmentAwareness.ui.controller.LIDARFilterAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.NormalEstimationAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.OcTreeBasicsAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.PointCloudAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.PolygonizerAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.RegionSegmentationAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.scene3D.RobotEnvironmentAwareness3DScene;
import us.ihmc.robotEnvironmentAwareness.ui.viewer.LidarFrameViewer;
import us.ihmc.robotEnvironmentAwareness.ui.viewer.REAMeshViewer;

public class LIDARBasedEnvironmentAwarenessUI
{
   private static final String CONFIGURATION_FILE_NAME = "./Configurations/defaultREAConfiguration.txt";

   private final RobotEnvironmentAwareness3DScene scene3D = new RobotEnvironmentAwareness3DScene();
   private final BorderPane mainPane;

   private final REAMessager reaMessager;
   private final REAMeshViewer reaMeshViewer;
   private final LidarFrameViewer lidarFrameViewer;

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

   private final Stage primaryStage;

   private LIDARBasedEnvironmentAwarenessUI(REAMessager reaMessager, Stage primaryStage) throws IOException
   {
      this.primaryStage = primaryStage;
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource("LIDARBasedEnvironmentAwarenessUI.fxml")); // temporary
      mainPane = loader.load();

      // Client
      this.reaMessager = reaMessager;

      lidarFrameViewer = new LidarFrameViewer(reaMessager);
      lidarFrameViewer.start();
      reaMeshViewer = new REAMeshViewer(reaMessager);
      reaMeshViewer.start();

      mainPane.setCenter(scene3D);

      scene3D.attachChild(pointCloudAnchorPaneController.getRoot());
      scene3D.attachChild(reaMeshViewer.getRoot());
      scene3D.attachChild(lidarFrameViewer.getRoot());


      File configurationFile = new File(CONFIGURATION_FILE_NAME);

      pointCloudAnchorPaneController.setConfigurationFile(configurationFile);
      pointCloudAnchorPaneController.attachREAMessager(reaMessager);
      pointCloudAnchorPaneController.bindControls();
      pointCloudAnchorPaneController.start();

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

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);
      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public void show()
   {
      primaryStage.show();
   }

   public void stop()
   {
      try
      {
         reaMessager.closeMessager();

         scene3D.stop();
         pointCloudAnchorPaneController.stop();
         reaMeshViewer.stop();
         lidarFrameViewer.stop();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public static LIDARBasedEnvironmentAwarenessUI creatIntraprocessUI(Stage primaryStage) throws IOException
   {
      REAMessager uiMessager = REAMessagerOverNetwork.createIntraprocess(NetworkPorts.REA_MODULE_UI_PORT, new REACommunicationKryoNetClassList());
      uiMessager.startMessager();
      return new LIDARBasedEnvironmentAwarenessUI(uiMessager, primaryStage);
   }

   public static LIDARBasedEnvironmentAwarenessUI creatRemoteUI(Stage primaryStage) throws IOException
   {
      REAMessager uiMessager = REAMessagerOverNetwork.createTCPClient("localhost", NetworkPorts.REA_MODULE_UI_PORT, new REACommunicationKryoNetClassList());
      uiMessager.startMessager();
      return new LIDARBasedEnvironmentAwarenessUI(uiMessager, primaryStage);
   }
}
