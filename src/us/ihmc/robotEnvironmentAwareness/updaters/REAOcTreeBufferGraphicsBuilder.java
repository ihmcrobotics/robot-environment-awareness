package us.ihmc.robotEnvironmentAwareness.updaters;

import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OctreeNodeData;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OctreeNodeMessage;

import javax.vecmath.Point3f;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class REAOcTreeBufferGraphicsBuilder
{

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> showInputScan;
   private final AtomicReference<Boolean> showBuffer;

   private final REAMessager reaMessager;


   public REAOcTreeBufferGraphicsBuilder(REAMessager reaMessager)
   {
      this.reaMessager = reaMessager;

      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      showBuffer = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsShowBuffer, false);
      showInputScan = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsShowInputScan, true);
   }

   public void update(NormalOcTree bufferOcTree, ScanCollection scanCollection)
   {
      if (enable.get()  && showBuffer.get())
      {
         float size = (float) OcTreeKeyConversionTools.computeNodeSize(bufferOcTree.getTreeDepth(), bufferOcTree.getResolution(), bufferOcTree.getTreeDepth());

         ArrayList<OctreeNodeData> nodesData = new ArrayList<>(bufferOcTree.size());

         for (NormalOcTreeNode node : bufferOcTree)
         {
            nodesData.add(new OctreeNodeData(node,  Integer.MIN_VALUE));
         }

         reaMessager.getPacketCommunicator().send(new OctreeNodeMessage(REAModuleAPI.BufferOctreeMessageID, nodesData, size));
      }

      if (showInputScan.get())
      {
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
            reaMessager.submitMessage(new REAMessage(REAModuleAPI.ScanPointsCollection, scannedPoints));
      }
   }


}
