package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.javaFXToolkit.shapes.MeshBuilder;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessager;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleAPI;

import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Created by adrien on 11/20/16.
 */
public class BufferOctreeMeshBuilder implements Runnable
{

   public interface BufferOctreeMeshBuilderListener
   {
      void meshAndMaterialChanged(Pair<Mesh, Material> meshMaterial);
   }

   private static final Color DEFAULT_BUFFER_COLOR = Color.RED;
   private static final double NODE_SCALE = 0.5;

   private final MeshBuilder bufferMeshBuilder = new MeshBuilder();
   private final Material bufferMaterial = new PhongMaterial(DEFAULT_BUFFER_COLOR);

   private BufferOctreeMeshBuilderListener listener;

   private AtomicReference<Boolean> renderingPaused = new AtomicReference<>(false);

   private final AtomicReference<ArrayList<Point3d>> bufferOctreeData;
   private final AtomicReference<Double> bufferOctreeNodeSize;

   public BufferOctreeMeshBuilder(REAMessager reaMessager, BufferOctreeMeshBuilderListener bufferOctreeMeshBuilderListener)
   {
      bufferOctreeNodeSize = reaMessager.createInput(REAModuleAPI.BufferOctreeNodeSize);
      bufferOctreeData = reaMessager.createInput(REAModuleAPI.BufferOctree);

      setListener(bufferOctreeMeshBuilderListener);
   }

   private void setListener(BufferOctreeMeshBuilderListener listener)
   {
      if (listener == null)
         throw new IllegalArgumentException("BufferOctreeMeshBuilderListener cannot be null");
      this.listener = listener;
   }

   @Override public void run()
   {

      if (renderingPaused.get())
         return;

      bufferMeshBuilder.clear();

      if (bufferOctreeData.get() != null && bufferOctreeNodeSize != null)
      {
         double size = bufferOctreeNodeSize.getAndSet(null).doubleValue();
         List<Point3d> nodePositions = bufferOctreeData.getAndSet(null);

         for (Point3d nodePosition : nodePositions)
            bufferMeshBuilder.addCube(NODE_SCALE * size, nodePosition.getX(), nodePosition.getY(), nodePosition.getZ());

         Pair<Mesh, Material> meshAndMaterial = new Pair<>(bufferMeshBuilder.generateMesh(), bufferMaterial);
         listener.meshAndMaterialChanged(meshAndMaterial);
      }


   }

   public void setRenderingPaused(boolean isPaused)
   {
      renderingPaused.set(isPaused);
   }

}
