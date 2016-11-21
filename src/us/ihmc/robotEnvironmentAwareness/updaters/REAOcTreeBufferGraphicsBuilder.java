package us.ihmc.robotEnvironmentAwareness.updaters;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.javaFXToolkit.shapes.MeshBuilder;
import us.ihmc.javaFXToolkit.shapes.MultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class REAOcTreeBufferGraphicsBuilder
{

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> showInputScan;
   private final AtomicReference<Boolean> showBuffer;

   private final REAMessager reaMessagerNet;

   private boolean hasClearedBufferGraphics = false;
   private boolean hasClearedScanGraphics = false;

   public REAOcTreeBufferGraphicsBuilder(REAMessager reaMessager, REAMessager reaMessagerNet)
   {
      this.reaMessagerNet = reaMessagerNet;

      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      showBuffer = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsShowBuffer, false);
      showInputScan = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsShowInputScan, true);

      TextureColorPalette1D scanColorPalette = new TextureColorPalette1D();
      scanColorPalette.setHueBased(1.0, 1.0);
   }

   public void update(NormalOcTree bufferOcTree, ScanCollection scanCollection)
   {
      if (!enable.get() || !showBuffer.get())
      {
         clearBufferGraphicsIfNeeded();
      }
      else
      {
         hasClearedBufferGraphics = false;

         double size = 0;
         ArrayList<Point3d> nodePositions = new ArrayList(bufferOcTree.getNumberOfNodes());
         for (NormalOcTreeNode node : bufferOcTree)
         {
            Point3d point = new Point3d();
            node.getCoordinate(point);
            nodePositions.add(point);
            size = node.getSize(); // Not sure how to access only one node
         }

         reaMessagerNet.submitMessage(new REAMessage(REAModuleAPI.BufferOctreeNodeSize, size));
         reaMessagerNet.submitMessage(new REAMessage(REAModuleAPI.BufferOctree, nodePositions));
      }

      if (!showInputScan.get())
      {
         clearScanGraphicsIfNeeded();
      }
      else
      {
         hasClearedScanGraphics = false;

         System.err.println("Nb of scans: " + scanCollection.getNumberOfScans());

         ArrayList<Point3f[]> scannedPoints = new ArrayList<>(scanCollection.getNumberOfScans());
         for (int scanIndex = 0; scanIndex < scanCollection.getNumberOfScans(); scanIndex++)
         {
            PointCloud pointCloud = scanCollection.getScan(scanIndex).getPointCloud();
            Point3f[] points = new Point3f[pointCloud.getNumberOfPoints()];

            for (int pointIndex = 0; pointIndex < pointCloud.getNumberOfPoints(); pointIndex++)
               points[pointIndex] = pointCloud.getPoint(pointIndex);

            scannedPoints.add(points);
         }

         if (!scannedPoints.isEmpty())
            reaMessagerNet.submitMessage(new REAMessage(REAModuleAPI.ScanPointsCollection, scannedPoints));
      }
   }

   private void clearBufferGraphicsIfNeeded()
   {
      if (hasClearedBufferGraphics)
         return;
      reaMessagerNet.submitMessage(new REAMessage(REAModuleAPI.BufferOctreeNodeSize, null));
      reaMessagerNet.submitMessage(new REAMessage(REAModuleAPI.BufferOctree, null));
      hasClearedBufferGraphics = true;
   }

   private void clearScanGraphicsIfNeeded()
   {
      if (hasClearedScanGraphics)
         return;
      reaMessagerNet.submitMessage(new REAMessage(REAModuleAPI.ScanPointsCollection, null));
      hasClearedScanGraphics = true;
   }
}
