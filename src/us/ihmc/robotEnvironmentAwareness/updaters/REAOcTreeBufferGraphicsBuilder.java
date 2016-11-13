package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicReference;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.graphics3DDescription.MeshDataGenerator;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.javaFXToolkit.shapes.MeshBuilder;
import us.ihmc.javaFXToolkit.shapes.MultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;

public class REAOcTreeBufferGraphicsBuilder
{
   private static final Color DEFAULT_BUFFER_COLOR = Color.RED;
   private static final double NODE_SCALE = 0.5;
   private static final float SCAN_POINT_SIZE = 0.0075f;

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> showInputScan;
   private final AtomicReference<Boolean> showBuffer;
   private final MeshBuilder bufferMeshBuilder = new MeshBuilder();
   private final Material bufferMaterial = new PhongMaterial(DEFAULT_BUFFER_COLOR);
   private final MultiColorMeshBuilder scanMeshBuilder;;

   private REAMessager outputMessager;

   private boolean hasClearedBufferGraphics = false;
   private boolean hasClearedScanGraphics = false;

   public REAOcTreeBufferGraphicsBuilder(REAMessageManager inputManager, REAMessager outputMessager)
   {
      this.outputMessager = outputMessager;

      enable = inputManager.createInput(REAModuleAPI.OcTreeEnable, false);
      showBuffer = inputManager.createInput(REAModuleAPI.OcTreeGraphicsShowBuffer, false);
      showInputScan = inputManager.createInput(REAModuleAPI.OcTreeGraphicsShowInputScan, true);

      TextureColorPalette1D scanColorPalette = new TextureColorPalette1D();
      scanColorPalette.setHueBased(1.0, 1.0);
      scanMeshBuilder = new MultiColorMeshBuilder(scanColorPalette);
   }

   public void update(NormalOcTree bufferOcTree, ScanCollection scanCollection)
   {
      if (!enable.get() || !showBuffer.get())
      {
         clearBufferGraphicsIfNeeded();
      }
      else
      {
         bufferMeshBuilder.clear();
         hasClearedBufferGraphics = false;
         
         for (NormalOcTreeNode node : bufferOcTree)
            bufferMeshBuilder.addCube(NODE_SCALE * node.getSize(), node.getX(), node.getY(), node.getZ());
         
         Pair<Mesh, Material> meshAndMaterial = new Pair<>(bufferMeshBuilder.generateMesh(), bufferMaterial);
         outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsBufferMesh, meshAndMaterial));
      }

      if (!showInputScan.get())
      {
         clearScanGraphicsIfNeeded();
      }
      else
      {
         scanMeshBuilder.clear();
         hasClearedScanGraphics = false;

         System.err.println("Nb of scans: " + scanCollection.getNumberOfScans());
         
         for (int scanIndex = 0; scanIndex < scanCollection.getNumberOfScans(); scanIndex++)
         {
            PointCloud pointCloud = scanCollection.getScan(scanIndex).getPointCloud();

            for (int pointIndex = 0; pointIndex < pointCloud.getNumberOfPoints(); pointIndex++)
            {
               double alpha = pointIndex / (double) pointCloud.getNumberOfPoints();
               Color color = Color.hsb(alpha * 240.0, 1.0, 1.0);
               scanMeshBuilder.addMesh(MeshDataGenerator.Tetrahedron(SCAN_POINT_SIZE), pointCloud.getPoint(pointIndex), color);
            }
         }

         Pair<Mesh, Material> meshAndMaterial = new Pair<>(scanMeshBuilder.generateMesh(), scanMeshBuilder.generateMaterial());
         outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsInputScanMesh, meshAndMaterial));
      }
   }

   private void clearBufferGraphicsIfNeeded()
   {
      if (hasClearedBufferGraphics)
         return;
      outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsBufferMesh, new Pair<Mesh, Material>(null, null)));
      hasClearedBufferGraphics = true;
   }

   private void clearScanGraphicsIfNeeded()
   {
      if (hasClearedScanGraphics)
         return;
      outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsInputScanMesh, new Pair<Mesh, Material>(null, null)));
      hasClearedScanGraphics = true;
   }
}
