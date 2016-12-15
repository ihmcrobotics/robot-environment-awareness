package us.ihmc.robotEnvironmentAwareness.ui.scene3D;

import javax.vecmath.Vector3d;

import javafx.application.Platform;
import javafx.event.Event;
import javafx.event.EventHandler;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.PerspectiveCamera;
import javafx.scene.SceneAntialiasing;
import javafx.scene.SubScene;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.transform.Translate;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;

public class RobotEnvironmentAwareness3DScene extends Pane
{
   private final SubScene subScene3D;
   private final Group rootNode = new Group();
   private final FocusBasedCameraMouseEventHandler cameraController;

   public RobotEnvironmentAwareness3DScene()
   {
      subScene3D = new SubScene(rootNode, -1, -1, true, SceneAntialiasing.DISABLED);
      subScene3D.setFill(Color.GRAY);
      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(0.3);
      worldCoordinateSystem.setMouseTransparent(true);
      rootNode.getChildren().add(worldCoordinateSystem);

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
      cameraController.enableShiftClickFocusTranslation();
      subScene3D.addEventHandler(Event.ANY, cameraController);
      rootNode.getChildren().add(cameraController.getFocusPointViz());

      this.getChildren().add(subScene3D);
      subScene3D.heightProperty().bind(this.heightProperty());
      subScene3D.widthProperty().bind(this.widthProperty());
   }

   public void attachChild(Node nodeToAttach)
   {
      if (Platform.isFxApplicationThread())
         attachChildNow(nodeToAttach);
      else
         Platform.runLater(() -> attachChildNow(nodeToAttach));
   }

   private void attachChildNow(Node nodeToAttach)
   {
      rootNode.getChildren().add(nodeToAttach);
   }

   public void stop()
   {
      // Nothing to stop for now
   }

   public Translate getCameraControllerTranslate()
   {
      return cameraController.getTranslate();
   }
}
