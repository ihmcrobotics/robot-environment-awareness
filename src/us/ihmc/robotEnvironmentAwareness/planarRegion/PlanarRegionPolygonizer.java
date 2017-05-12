package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHull;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullCollection;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullPruningFilteringTools;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataExporter;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public abstract class PlanarRegionPolygonizer
{
   public static PlanarRegionsList createPlanarRegionsList(List<PlanarRegionSegmentationRawData> rawData,
         ConcaveHullFactoryParameters concaveHullFactoryParameters, PolygonizerParameters polygonizerParameters)
   {
      return createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters, null);
   }

   public static PlanarRegionsList createPlanarRegionsList(List<PlanarRegionSegmentationRawData> rawData,
         ConcaveHullFactoryParameters concaveHullFactoryParameters, PolygonizerParameters polygonizerParameters,
         PlanarRegionSegmentationDataExporter dataExporter)
   {
      return new PlanarRegionsList(createPlanarRegions(rawData, concaveHullFactoryParameters, polygonizerParameters, dataExporter));
   }

   private static List<PlanarRegion> createPlanarRegions(List<PlanarRegionSegmentationRawData> rawData,
         ConcaveHullFactoryParameters concaveHullFactoryParameters, PolygonizerParameters polygonizerParameters,
         PlanarRegionSegmentationDataExporter dataExporter)
   {
      return rawData.parallelStream()
                    .filter(data -> data.size() >= polygonizerParameters.getMinNumberOfNodes())
                    .map(data -> createPlanarRegion(data, concaveHullFactoryParameters, polygonizerParameters, dataExporter))
                    .filter(region -> region != null)
                    .collect(Collectors.toList());
   }

   private static PlanarRegion createPlanarRegion(PlanarRegionSegmentationRawData rawData,
         ConcaveHullFactoryParameters concaveHullFactoryParameters, PolygonizerParameters polygonizerParameters,
         PlanarRegionSegmentationDataExporter dataExporter)
   {
      try
      {
         // First compute the set of concave hulls for this region
         ConcaveHullCollection concaveHullCollection = SimpleConcaveHullFactory.createConcaveHullCollection(rawData.getPointCloudInPlane(),
               concaveHullFactoryParameters);

         // Apply some simple filtering to reduce the number of vertices and hopefully the number of convex polygons.
         double shallowAngleThreshold = polygonizerParameters.getShallowAngleThreshold();
         double peakAngleThreshold = polygonizerParameters.getPeakAngleThreshold();
         double lengthThreshold = polygonizerParameters.getLengthThreshold();

         for (int i = 0; i < 5; i++)
         {
            ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHullCollection);
            ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHullCollection);
         }

         // Decompose the concave hulls into convex polygons
         double depthThreshold = polygonizerParameters.getDepthThreshold();
         List<ConvexPolygon2D> decomposedPolygons = new ArrayList<>();
         ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHullCollection, depthThreshold, decomposedPolygons);

         // Pack the data in PlanarRegion
         RigidBodyTransform transformToWorld = rawData.getTransformFromLocalToWorld();
         List<Point2D[]> concaveHullsVertices = new ArrayList<>();
         for (ConcaveHull concaveHull : concaveHullCollection)
            concaveHullsVertices.add(concaveHull.getConcaveHullVertices().toArray(new Point2D[0]));
         
         PlanarRegion planarRegion = new PlanarRegion(transformToWorld, concaveHullsVertices, decomposedPolygons);
         planarRegion.setRegionId(rawData.getRegionId());

         return planarRegion;
      }
      catch (RuntimeException e)
      {
         if (dataExporter == null)
         {
            e.printStackTrace();
         }
         else
         {
            PrintTools.error("Caught following exception: " + e.getMessage() + ", exporting segmentation data.");
            dataExporter.exportSegmentationRawData(rawData);
         }
         return null;
      }
   }
}
