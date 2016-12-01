package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.communication.packets.LidarScanMessage;
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
   private static final float SCAN_POINT_SIZE = 0.0075f;

   private final JavaFXMultiColorMeshBuilder multiColorMeshBuilder;

   private final AtomicReference<LidarScanMessage> lidarScanToRender;

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> showInputScan;

   private final AtomicReference<Pair<Mesh, Material>> meshAndMaterialToRender = new AtomicReference<>(null);

   private boolean hasClearedScanGraphics = false;

   private final REAMessager reaMessager;

   public ScanMeshBuilder(REAMessager reaMessager)
   {
      this.reaMessager = reaMessager;
      // over network
      lidarScanToRender = reaMessager.createInput(REAModuleAPI.LidarScanState);

      // local
      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      showInputScan = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsShowInputScan, true);

      TextureColorPalette1D scanColorPalette = new TextureColorPalette1D();
      scanColorPalette.setHueBased(1.0, 1.0);
      multiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(scanColorPalette);
   }

   @Override
   public void run()
   {
      if (!enable.get() || !showInputScan.get())
      {
         clearScanGraphicsIfNeeded();
         return;
      }
      multiColorMeshBuilder.clear();

      LidarScanMessage lidarScan = lidarScanToRender.getAndSet(null);
      reaMessager.submitStateRequest(REAModuleAPI.RequestLidarScan);

      if (lidarScan == null)
         return;

      Point3d scanPoint = new Point3d();
      int numberOfScanPoints = lidarScan.getNumberOfScanPoints();

      for (int pointIndex = 0; pointIndex < numberOfScanPoints; pointIndex++)
      {
         double alpha = pointIndex / (double) numberOfScanPoints;
         Color color = Color.hsb(alpha * 240.0, 1.0, 1.0);
         lidarScan.getScanPoint(pointIndex, scanPoint);
         multiColorMeshBuilder.addMesh(MeshDataGenerator.Tetrahedron(SCAN_POINT_SIZE), scanPoint, color);
      }

      Pair<Mesh, Material> meshAndMaterial = new Pair<>(multiColorMeshBuilder.generateMesh(), multiColorMeshBuilder.generateMaterial());
      meshAndMaterialToRender.set(meshAndMaterial);

      hasClearedScanGraphics = false;
   }

   private void clearScanGraphicsIfNeeded()
   {
      if (hasClearedScanGraphics)
         return;

      multiColorMeshBuilder.clear();
      meshAndMaterialToRender.set(new Pair<>(null, null));
      hasClearedScanGraphics = true;
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
