package us.ihmc.robotEnvironmentAwareness.ui;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.event.EventHandler;
import javafx.geometry.Orientation;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.control.SplitPane;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.robotEnvironmentAwareness.communication.LidarSimulationNetClassList;
import us.ihmc.robotEnvironmentAwareness.ui.ocTree.OcTreeUIControlFactory;

public class FootstepPlannerUI extends Application
{
   public static final boolean CREATE_HEIGHT_MAP_VIEWER = false;
   public static final boolean CREATE_POINT_CLOUD_VIEWER = true;
   public static final boolean CREATE_OCTREE_VIEWER = true;
   public static final boolean CREATE_LIDAR_VIEWER = true;

   private final PacketCommunicator packetCommunicator;
   private FootstepPlannerUI3DPane ui3dpane;
   private FootstepPlannerUIInteractionPane uiInteractionPane;

   public FootstepPlannerUI()
   {
      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient("localhost", NetworkPorts.BEHAVIOUR_MODULE_PORT,
            new LidarSimulationNetClassList());
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle("Foostep Planner UI");
      primaryStage.setMaximized(true);

      Group root = new Group();
      Scene scene = new Scene(root, 600, 400);

      packetCommunicator.connect();

      ui3dpane = new FootstepPlannerUI3DPane(600, 400, packetCommunicator);
      PointCloudPropertyControlFactory pointCloudUIControlFactory = ui3dpane.getPointCloudUIControlFactory();
      HeightMapPropertyControlFactory heightMapUIControlFactory = ui3dpane.getHeightMapUIControlFactory();
      OcTreeUIControlFactory ocTreeUIControlFactory = ui3dpane.getOcTreeUIControlFactory();
      uiInteractionPane = new FootstepPlannerUIInteractionPane(pointCloudUIControlFactory, heightMapUIControlFactory, ocTreeUIControlFactory);

      SplitPane mainPane = new SplitPane();
      mainPane.prefHeightProperty().bind(scene.heightProperty());
      mainPane.prefWidthProperty().bind(scene.widthProperty());
      mainPane.setOrientation(Orientation.VERTICAL);
      mainPane.getItems().addAll(ui3dpane, uiInteractionPane);
      mainPane.setDividerPositions(0.75, 0.25);
      root.getChildren().add(mainPane);

      primaryStage.setScene(scene);
      primaryStage.show();

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
         if (ui3dpane != null)
            ui3dpane.stop();
         if (uiInteractionPane != null)
            uiInteractionPane.stop();
         Platform.exit();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public static void main(String[] args)
   {
      Application.launch(args);
   }
}
