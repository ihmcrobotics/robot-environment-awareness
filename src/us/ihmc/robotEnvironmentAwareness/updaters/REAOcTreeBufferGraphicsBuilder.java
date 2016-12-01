package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3f;

import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.communication.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;

public class REAOcTreeBufferGraphicsBuilder
{

   private final AtomicReference<Boolean> isBufferRequested;
   private final AtomicReference<Boolean> isInputScanRequested;

   private final REAMessager reaMessager;

   public REAOcTreeBufferGraphicsBuilder(REAMessager reaMessager)
   {
      this.reaMessager = reaMessager;

      isBufferRequested = reaMessager.createInput(REAModuleAPI.RequestBuffer, false);
      isInputScanRequested = reaMessager.createInput(REAModuleAPI.RequestLidarScan, false);
   }

   public void update(NormalOcTree bufferOcTree, ScanCollection scanCollection)
   {
      if (isBufferRequested.getAndSet(false))
      {
         NormalOcTreeMessage normalOcTreeMessage = OcTreeMessageConverter.convertToMessage(bufferOcTree);
         reaMessager.submitMessage(REAModuleAPI.BufferState, normalOcTreeMessage);
      }

      if (isInputScanRequested.getAndSet(false))
      {
         List<Point3f[]> scannedPoints = new ArrayList<>(scanCollection.getNumberOfScans());
         for (int scanIndex = 0; scanIndex < scanCollection.getNumberOfScans(); scanIndex++)
         {
            PointCloud pointCloud = scanCollection.getScan(scanIndex).getPointCloud();
            Point3f[] points = new Point3f[pointCloud.getNumberOfPoints()];

            for (int pointIndex = 0; pointIndex < pointCloud.getNumberOfPoints(); pointIndex++)
               points[pointIndex] = pointCloud.getPoint(pointIndex);

            scannedPoints.add(points);
         }

         if (!scannedPoints.isEmpty())
            reaMessager.submitMessage(REAModuleAPI.LidarScanState, scannedPoints);
      }
   }

}
