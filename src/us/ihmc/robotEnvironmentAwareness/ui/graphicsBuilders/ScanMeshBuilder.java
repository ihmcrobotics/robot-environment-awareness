package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3f;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.graphics3DDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;

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

   private final JavaFXMultiColorMeshBuilder multiColorMeshBuilder;
   private ScanMeshBuilderListener listener;


   private final AtomicReference<ArrayList<Point3f[]>> scanInputMeshToRender;

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> showInputScan;

   private boolean hasClearedScanGraphics = false;


   public ScanMeshBuilder(REAMessager reaMessager, ScanMeshBuilderListener scanMeshBuilderListener)
   {
      // over network
      scanInputMeshToRender = reaMessager.createInput(REAModuleAPI.ScanPointsCollection);

      // local
      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      showInputScan = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsShowInputScan, true);

      TextureColorPalette1D scanColorPalette = new TextureColorPalette1D();
      scanColorPalette.setHueBased(1.0, 1.0);
      multiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(scanColorPalette);

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
      if (!enable.get() || !showInputScan.get())
      {
         clearScanGraphicsIfNeeded();
      }
      else
      {
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

            hasClearedScanGraphics = false;
         }
      }
   }

   private void clearScanGraphicsIfNeeded()
   {
      if (hasClearedScanGraphics)
         return;

      multiColorMeshBuilder.clear();

      Pair<Mesh, Material> meshAndMaterial = new Pair<>(multiColorMeshBuilder.generateMesh(), multiColorMeshBuilder.generateMaterial());
      listener.scanMeshAndMaterialChanged(meshAndMaterial);

      hasClearedScanGraphics = true;
   }

}
