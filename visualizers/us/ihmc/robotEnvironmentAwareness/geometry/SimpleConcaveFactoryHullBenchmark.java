package us.ihmc.robotEnvironmentAwareness.geometry;

import java.io.File;
import java.io.IOException;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataImporter;

/**
 * Simple infinite loop on the {@link SimpleConcaveHullFactory} for convenience for profiling the algorithm.
 * @author Sylvain Bertrand
 *
 */
public class SimpleConcaveFactoryHullBenchmark
{
   public static void main(String[] args) throws IOException
   {
      PlanarRegionSegmentationDataImporter dataImporter = new PlanarRegionSegmentationDataImporter(new File("Data/20161210_185643_PlanarRegionSegmentation_Atlas_CB"));
      dataImporter.loadPlanarRegionSegmentationData();
      List<PlanarRegionSegmentationMessage> planarRegionSegmentationData = dataImporter.getPlanarRegionSegmentationData();
      PolygonizerParameters parameters = new PolygonizerParameters();

      while (true)
      {
         for (PlanarRegionSegmentationMessage planarRegionSegmentationMessage : planarRegionSegmentationData)
         {
            List<Point2d> pointsInPlane = PolygonizerTools.extractPointsInPlane(planarRegionSegmentationMessage);
            SimpleConcaveHullFactory.createConcaveHull(pointsInPlane, parameters.getConcaveHullThreshold());
         }
      }
   }
}
