package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.transform.Affine;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoxMessage;

public class BoundingBoxMeshBuilder implements Runnable
{
   private static final Color OCTREE_BBX_COLOR = new Color(Color.DARKGREY.getRed(), Color.DARKGREY.getGreen(), Color.DARKGREY.getBlue(), 0.0);

   private final AtomicReference<Boolean> enable;

   private final Material material = new PhongMaterial(OCTREE_BBX_COLOR);

   private final AtomicReference<Boolean> showOcTreeBoundingBox;
   private final AtomicReference<BoxMessage> boundingBoxState;
   private final AtomicReference<Box> boundingBoxToRender = new AtomicReference<>(null);

   private final REAUIMessager uiMessager;

   public BoundingBoxMeshBuilder(REAUIMessager uiMessager)
   {
      this.uiMessager = uiMessager;
      enable = uiMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      showOcTreeBoundingBox = uiMessager.createInput(REAModuleAPI.OcTreeGraphicsBoundingBoxShow, false);

      boundingBoxState = uiMessager.createInput(REAModuleAPI.BoundingBoxState);
   }

   @Override
   public void run()
   {
      BoxMessage newMessage = boundingBoxState.getAndSet(null);

      if (!enable.get() || showOcTreeBoundingBox.get())
         return;

      uiMessager.submitStateRequestToModule(REAModuleAPI.RequestBoundingBox);

      if (newMessage == null)
         return;

      Vector3f boxSize = newMessage.getSize();
      Quat4f boxOrientation = newMessage.getOrientation();
      Point3f boxCenter = newMessage.getCenter();

      Box box = new Box();
      box.setWidth(boxSize.getX());
      box.setHeight(boxSize.getY());
      box.setDepth(boxSize.getZ());
      Affine boxPose = JavaFXTools.createAffineFromQuaternionAndTuple(boxOrientation, boxCenter);
      box.getTransforms().add(boxPose);
      box.setMaterial(material);
      boundingBoxToRender.set(box);
   }

   public boolean hasBoundingBoxToRender()
   {
      return boundingBoxToRender.get() != null;
   }

   public Box getBoundingBox()
   {
      return boundingBoxToRender.get();
   }
}
