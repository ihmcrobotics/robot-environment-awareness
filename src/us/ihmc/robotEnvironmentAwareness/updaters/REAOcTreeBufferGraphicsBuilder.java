package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicReference;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.javaFXToolkit.shapes.MeshBuilder;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;

public class REAOcTreeBufferGraphicsBuilder
{
   private static final Color DEFAULT_COLOR = Color.RED;
   private static final double NODE_SCALE = 0.5;

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> showBuffer;
   private final MeshBuilder meshBuilder = new MeshBuilder();
   private final Material material = new PhongMaterial(DEFAULT_COLOR);

   private REAMessager outputMessager;

   private boolean hasCleared = false;

   public REAOcTreeBufferGraphicsBuilder(REAMessageManager inputManager, REAMessager outputMessager)
   {
      this.outputMessager = outputMessager;

      enable = inputManager.createInput(REAModuleAPI.OcTreeEnable, false);
      showBuffer = inputManager.createInput(REAModuleAPI.OcTreeGraphicsShowBuffer, false);
   }

   public void update(NormalOcTree bufferOcTree)
   {
      if (!enable.get() || !showBuffer.get())
      {
         if (!hasCleared)
         {
            outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsBufferMesh, new Pair<Mesh, Material>(null, null)));
            hasCleared = true;
         }
         return;
      }

      meshBuilder.clear();
      hasCleared = false;

      for (NormalOcTreeNode node : bufferOcTree)
         meshBuilder.addCube(NODE_SCALE * node.getSize(), node.getX(), node.getY(), node.getZ());

      Pair<Mesh, Material> meshAndMaterial = new Pair<>(meshBuilder.generateMesh(), material);
      outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsBufferMesh, meshAndMaterial));
   }
}
