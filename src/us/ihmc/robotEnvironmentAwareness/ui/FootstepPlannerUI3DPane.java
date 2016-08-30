package us.ihmc.robotEnvironmentAwareness.ui;

import javax.vecmath.Vector3d;

import javafx.event.Event;
import javafx.event.EventHandler;
import javafx.scene.AmbientLight;
import javafx.scene.Group;
import javafx.scene.PerspectiveCamera;
import javafx.scene.SceneAntialiasing;
import javafx.scene.SubScene;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.transform.Translate;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.robotEnvironmentAwareness.communication.LidarPosePacket;
import us.ihmc.robotEnvironmentAwareness.ui.ocTree.OcTreeUIControlFactory;
import us.ihmc.robotEnvironmentAwareness.ui.ocTree.OcTreeUIController;
import us.ihmc.robotEnvironmentAwareness.ui.ocTree.OcTreeViewer;

public class FootstepPlannerUI3DPane extends Pane
{
   private final SubScene subScene3D;
   private final Group rootNode = new Group();
   private final FocusBasedCameraMouseEventHandler cameraController;
   private final PointCloudViewer pointCloudViewer;
   private final HeightMapViewer heightMapViewer;
   private final OcTreeViewer ocTreeViewer;
   private final LidarFrameViewer lidarFrameViewer;

   public FootstepPlannerUI3DPane(double width, double height, PacketCommunicator packetCommunicator)
   {
      setPrefWidth(width);
      setPrefHeight(height);

      subScene3D = new SubScene(rootNode, width, height, true, SceneAntialiasing.DISABLED);
      subScene3D.setFill(Color.GRAY);
      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(0.3);
      rootNode.getChildren().add(worldCoordinateSystem);

      if (FootstepPlannerUI.CREATE_POINT_CLOUD_VIEWER)
      {
         pointCloudViewer = new PointCloudViewer();
         pointCloudViewer.start();
         rootNode.getChildren().add(pointCloudViewer);
         packetCommunicator.attachListener(PointCloudWorldPacket.class, pointCloudViewer.getPointCloudWorldPacketConsumer());
      }
      else
      {
         pointCloudViewer = null;
      }

      if (FootstepPlannerUI.CREATE_HEIGHT_MAP_VIEWER)
      {
         heightMapViewer = new HeightMapViewer();
         heightMapViewer.start();
         rootNode.getChildren().add(heightMapViewer);
         packetCommunicator.attachListener(PointCloudWorldPacket.class, heightMapViewer.getPointCloudWorldPacketConsumer());
      }
      else
      {
         heightMapViewer = null;
      }

      if (FootstepPlannerUI.CREATE_OCTREE_VIEWER)
      {
         boolean enable = true;
         OcTreeUIController ocTreeUIController = new OcTreeUIController(enable);
         ocTreeUIController.attachListeners(packetCommunicator);

         ocTreeViewer = new OcTreeViewer(ocTreeUIController, enable);
         ocTreeViewer.start();
         rootNode.getChildren().add(ocTreeViewer);
      }
      else
      {
         ocTreeViewer = null;
      }

      if (FootstepPlannerUI.CREATE_LIDAR_VIEWER)
      {
         lidarFrameViewer = new LidarFrameViewer();
         rootNode.getChildren().add(lidarFrameViewer);
         packetCommunicator.attachListener(LidarPosePacket.class, lidarFrameViewer.createLidarPosePacketConsumer());
      }
      else
      {
         lidarFrameViewer = null;
      }

      PerspectiveCamera camera = new PerspectiveCamera(true);
      camera.setNearClip(0.05);
      camera.setFarClip(50.0);
      subScene3D.setCamera(camera);

      subScene3D.setOnMouseClicked(new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            subScene3D.requestFocus();
         }
      });

      Vector3d up = new Vector3d(0.0, 0.0, 1.0);
      cameraController = new FocusBasedCameraMouseEventHandler(subScene3D.widthProperty(), subScene3D.heightProperty(), camera, up);
      subScene3D.addEventHandler(Event.ANY, cameraController);
      rootNode.getChildren().add(cameraController.getFocusPointViz());

      this.getChildren().add(subScene3D);
      subScene3D.heightProperty().bind(this.heightProperty());
      subScene3D.widthProperty().bind(this.widthProperty());
      
      AmbientLight ambientLight = new AmbientLight(Color.WHITE.darker());
      ambientLight.setTranslateX(0.0);
      ambientLight.setTranslateY(0.0);
      ambientLight.setTranslateZ(1.0);
   }

   public void stop()
   {
      if (pointCloudViewer != null)
         pointCloudViewer.stop();
      if (heightMapViewer != null)
         heightMapViewer.stop();
      if (ocTreeViewer != null)
         ocTreeViewer.stop();
   }

   public Translate getCameraControllerTranslate()
   {
      return cameraController.getTranslate();
   }

   public PointCloudPropertyControlFactory getPointCloudUIControlFactory()
   {
      return pointCloudViewer == null ? null : pointCloudViewer.uiControlFactory();
   }

   public HeightMapPropertyControlFactory getHeightMapUIControlFactory()
   {
      if (heightMapViewer != null)
         return heightMapViewer.uiControlFactory();
      else
         return null;
   }

   public OcTreeUIControlFactory getOcTreeUIControlFactory()
   {
      return ocTreeViewer == null ? null : ocTreeViewer.uiControlFactory();
   }
}
