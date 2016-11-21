package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.graphics3DDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.shapes.MultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessager;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleAPI;
import us.ihmc.simulationconstructionset.gui.BodePlotConstructor;

import javax.vecmath.Point3f;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Created by adrien on 11/20/16.
 */
public class ScanMeshBuilder implements Runnable
{

   public interface ScanMeshBuilderListener
   {
      void scanMeshAndMaterialChanged(Pair<Mesh, Material> meshMaterial);
   }

   private static final float SCAN_POINT_SIZE = 0.0075f;

   private final MultiColorMeshBuilder multiColorMeshBuilder;
   private ScanMeshBuilderListener listener;

   private AtomicReference<Boolean> renderingPaused = new AtomicReference<>(false);

   private final AtomicReference<ArrayList<Point3f[]>> scanInputMeshToRender;

   public ScanMeshBuilder(REAMessager reaMessager, ScanMeshBuilderListener scanMeshBuilderListener)
   {
      scanInputMeshToRender = reaMessager.createInput(REAModuleAPI.ScanPointsCollection);

      TextureColorPalette1D scanColorPalette = new TextureColorPalette1D();
      scanColorPalette.setHueBased(1.0, 1.0);
      multiColorMeshBuilder = new MultiColorMeshBuilder(scanColorPalette);

      setListener(scanMeshBuilderListener);
   }

   private void setListener(ScanMeshBuilderListener listener)
   {
      if (listener == null)
         throw new IllegalArgumentException("ScanMeshBuilderListener cannot be null");
      this.listener = listener;
   }

   @Override public void run()
   {

      if (renderingPaused.get())
         return;

      multiColorMeshBuilder.clear();

      if (scanInputMeshToRender.get() != null)
      {
         ArrayList<Point3f[]> pointClouds = scanInputMeshToRender.getAndSet(null);

         for (int scanIndex = 0; scanIndex < pointClouds.size(); scanIndex++)
         {
            Point3f[] pointCloud = pointClouds.get(scanIndex);

            for (int pointIndex = 0; pointIndex < pointCloud.length; pointIndex++)
            {
               double alpha = pointIndex / (double) pointCloud.length;
               Color color = Color.hsb(alpha * 240.0, 1.0, 1.0);
               multiColorMeshBuilder.addMesh(MeshDataGenerator.Tetrahedron(SCAN_POINT_SIZE), pointCloud[pointIndex], color);
            }
         }

         Pair<Mesh, Material> meshAndMaterial = new Pair<>(multiColorMeshBuilder.generateMesh(), multiColorMeshBuilder.generateMaterial());
         listener.scanMeshAndMaterialChanged(meshAndMaterial);
      }
   }

   public void setRenderingPaused(boolean isPaused)
   {
      renderingPaused.set(isPaused);
   }

}
