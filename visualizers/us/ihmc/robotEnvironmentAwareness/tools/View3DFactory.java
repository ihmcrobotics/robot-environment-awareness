package us.ihmc.robotEnvironmentAwareness.tools;

import javax.vecmath.Vector3d;

import javafx.event.Event;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.PerspectiveCamera;
import javafx.scene.PointLight;
import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import javafx.scene.paint.Color;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;

public class View3DFactory
{
   private final Group root = new Group();
   private final Scene scene;
   
   public View3DFactory(double width, double height)
   {
      this(width, height, true);
   }

   public View3DFactory(double width, double height, boolean depthBuffer)
   {
      this(width, height, depthBuffer, SceneAntialiasing.BALANCED);
   }

   public View3DFactory(double width, double height, boolean depthBuffer, SceneAntialiasing antiAliasing)
   {
      scene = new Scene(root, width, height, depthBuffer, antiAliasing);
      scene.setFill(Color.GRAY);
   }

   public void addPointLight(double x, double y, double z)
   {
      addPointLight(x, y, z, Color.WHITE);
   }

   public void addPointLight(double x, double y, double z, Color color)
   {
      PointLight light = new PointLight(color);
      light.setTranslateX(x);
      light.setTranslateY(y);
      light.setTranslateZ(z);
      addNodeToView(light);
   }

   public void addCameraController()
   {
      addCameraController(0.05, 50.0);
   }

   public void addCameraController(double nearClip, double farClip)
   {
      PerspectiveCamera camera = new PerspectiveCamera(true);
      camera.setNearClip(nearClip);
      camera.setFarClip(farClip);
      scene.setCamera(camera);

      Vector3d up = new Vector3d(0.0, 0.0, 1.0);
      FocusBasedCameraMouseEventHandler cameraController = new FocusBasedCameraMouseEventHandler(scene.widthProperty(), scene.heightProperty(), camera, up);
      scene.addEventHandler(Event.ANY, cameraController);
      addNodeToView(cameraController.getFocusPointViz());
   }

   public void addWorldCoordinateSystem(double arrowLength)
   {
      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(arrowLength);
      addNodeToView(worldCoordinateSystem);
   }

   public void addNodesToView(Iterable<Node> nodes)
   {
      nodes.forEach(this::addNodeToView);
   }

   public void addNodeToView(Node node)
   {
      root.getChildren().add(node);
   }

   public void setRootMouseTransparent(boolean value)
   {
      root.setMouseTransparent(value);
   }

   public Group getRoot()
   {
      return root;
   }

   public Scene getScene()
   {
      return scene;
   }
}
