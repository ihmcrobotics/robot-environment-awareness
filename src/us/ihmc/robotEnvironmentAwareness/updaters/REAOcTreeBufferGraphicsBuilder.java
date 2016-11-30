package us.ihmc.robotEnvironmentAwareness.updaters;

import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.robotEnvironmentAwareness.communication.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;

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
      if (enable.get() && showBuffer.get() && bufferOcTree.getRoot() != null)
      {
         NormalOcTreeMessage normalOcTreeMessage = OcTreeMessageConverter.convertToMessage(bufferOcTree);
         normalOcTreeMessage.messageID = REAModuleAPI.BufferOctreeMessageID;
         reaMessager.getPacketCommunicator().send(normalOcTreeMessage);
      }

      if (showInputScan.get())
      {
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
