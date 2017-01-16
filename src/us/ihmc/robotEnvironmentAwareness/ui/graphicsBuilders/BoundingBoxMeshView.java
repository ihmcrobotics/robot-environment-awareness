package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import javafx.application.Platform;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoxMessage;

public class BoundingBoxMeshView extends MeshView implements Runnable
{
   private static final Color OCTREE_BBX_COLOR = new Color(Color.DARKGREY.getRed(), Color.DARKGREY.getGreen(), Color.DARKGREY.getBlue(), 0.0);

   private final Material material = new PhongMaterial(OCTREE_BBX_COLOR);

   private final AtomicReference<Boolean> showOcTreeBoundingBox;
   private final AtomicReference<BoxMessage> boundingBoxState;

   private final REAUIMessager uiMessager;

   public BoundingBoxMeshView(REAUIMessager uiMessager)
   {
      this.uiMessager = uiMessager;
      showOcTreeBoundingBox = uiMessager.createInput(REAModuleAPI.UIOcTreeBoundingBoxShow, false);
      boundingBoxState = uiMessager.createInput(REAModuleAPI.OcTreeBoundingBoxState);
      setMaterial(material);
   }

   @Override
   public void run()
   {
      BoxMessage newMessage = boundingBoxState.getAndSet(null);

      if (showOcTreeBoundingBox.get())
      {
         uiMessager.submitStateRequestToModule(REAModuleAPI.RequestBoundingBox);
      }
      else
      {
         Platform.runLater(() -> setMesh(null));
         return;
      }

      if (newMessage == null)
         return;

      if (newMessage.isEmpty())
      {
         Platform.runLater(() -> setMesh(null));
         return;
      }

      Vector3f boxSize = newMessage.getSize();
      Quat4f boxOrientation = newMessage.getOrientation();
      Point3f boxCenter = newMessage.getCenter();

      MeshDataHolder boxMeshDataHolder = MeshDataGenerator.Cube(boxSize.getX(), boxSize.getY(), boxSize.getZ(), true);
      boxMeshDataHolder = MeshDataHolder.rotate(boxMeshDataHolder, boxOrientation);
      boxMeshDataHolder = MeshDataHolder.translate(boxMeshDataHolder, boxCenter);
      Mesh boxMesh = JavaFXMeshDataInterpreter.interpretMeshData(boxMeshDataHolder);

      Platform.runLater(() -> setMesh(boxMesh));
   }
}
