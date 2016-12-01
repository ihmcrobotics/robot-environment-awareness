package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;

/**
 * Created by adrien on 11/20/16.
 */
public class BufferOctreeMeshBuilder implements Runnable
{
   private static final Color DEFAULT_BUFFER_COLOR = Color.RED;
   private static final double NODE_SCALE = 0.5;

   private final JavaFXMeshBuilder bufferMeshBuilder = new JavaFXMeshBuilder();
   private final Material bufferMaterial = new PhongMaterial(DEFAULT_BUFFER_COLOR);

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> showBuffer;
   private final AtomicReference<NormalOcTreeMessage> bufferState;

   private final AtomicReference<Pair<Mesh, Material>> meshAndMaterialToRender = new AtomicReference<>(null);

   private boolean hasClearedBufferGraphics = false;
   private final REAMessager reaMessager;

   public BufferOctreeMeshBuilder(REAMessager reaMessager)
   {
      this.reaMessager = reaMessager;
      // local
      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      showBuffer = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsShowBuffer, true);
      bufferState = reaMessager.createInput(REAModuleAPI.BufferState);
   }

   @Override
   public void run()
   {
      NormalOcTreeMessage newBufferState = bufferState.getAndSet(null);

      if (!enable.get() || !showBuffer.get())
      {
         if (hasClearedBufferGraphics)
            return;
         bufferMeshBuilder.clear();
         meshAndMaterialToRender.set(new Pair<>(null, null));
         hasClearedBufferGraphics = true;
         return;
      }

      reaMessager.submitStateRequest(REAModuleAPI.RequestBuffer);

      bufferMeshBuilder.clear();

      if (newBufferState == null)
         return;

      UIOcTree uiOcTree = new UIOcTree(newBufferState);

      for (UIOcTreeNode uiOcTreeNode : uiOcTree)
      {
         Point3d nodeCenter = new Point3d(uiOcTreeNode.getX(), uiOcTreeNode.getY(), uiOcTreeNode.getZ());
         bufferMeshBuilder.addTetrahedron(NODE_SCALE * uiOcTreeNode.getSize(), nodeCenter);
      }

      meshAndMaterialToRender.set(new Pair<>(bufferMeshBuilder.generateMesh(), bufferMaterial));
      hasClearedBufferGraphics = false;
   }

   public boolean hasNewMeshAndMaterial()
   {
      return meshAndMaterialToRender.get() != null;
   }

   public Pair<Mesh, Material> pollMeshAndMaterial()
   {
      return meshAndMaterialToRender.getAndSet(null);
   }
}
