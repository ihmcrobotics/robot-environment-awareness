package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.OcTreeNodePlanarRegion;
import us.ihmc.robotEnvironmentAwareness.planarRegion.OcTreeNodePlanarRegionCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionConcaveHull;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionConvexPolygons;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionIntersectionCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.LineSegment3d;

public class REAPlanarRegionFeatureUpdater implements RegionFeaturesProvider
{
   private static final String segmentationTimeReport = "Segmentation took: ";
   private static final String intersectionsTimeReport = "Processing intersections took: ";

   private TimeReporter timeReporter = new TimeReporter(this);
   private final NormalOcTree octree;

   private final OcTreeNodePlanarRegionCalculator planarRegionCalculator = new OcTreeNodePlanarRegionCalculator();
   private final PlanarRegionIntersectionCalculator intersectionCalculator = new PlanarRegionIntersectionCalculator();

   private Map<OcTreeNodePlanarRegion, PlanarRegionConcaveHull> concaveHulls = null;
   private Map<OcTreeNodePlanarRegion, PlanarRegionConvexPolygons> convexPolygons = null;

   private final AtomicReference<Boolean> isOcTreeEnabled;
   private final AtomicReference<Boolean> enableSegmentation;
   private final AtomicReference<Boolean> clearSegmentation;
   private final AtomicReference<Boolean> enablePolygonizer;
   private final AtomicReference<Boolean> enableIntersectionCalulator;
   private final AtomicReference<PlanarRegionSegmentationParameters> planarRegionSegmentationParameters;
   private final AtomicReference<IntersectionEstimationParameters> intersectionEstimationParameters;
   private final AtomicReference<PolygonizerParameters> polygonizerParameters;
   private final REAMessager reaMessager;

   public REAPlanarRegionFeatureUpdater(NormalOcTree octree, REAMessager reaMessager)
   {
      this.octree = octree;
      this.reaMessager = reaMessager;

      isOcTreeEnabled = reaMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      enableSegmentation = reaMessager.createInput(REAModuleAPI.OcTreePlanarRegionSegmentationEnable, false);
      clearSegmentation = reaMessager.createInput(REAModuleAPI.OcTreePlanarRegionSegmentationClear, false);
      enablePolygonizer = reaMessager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerEnable, false);
      enableIntersectionCalulator = reaMessager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionEnable, false);
      planarRegionSegmentationParameters = reaMessager.createInput(REAModuleAPI.OcTreePlanarRegionSegmentationParameters, new PlanarRegionSegmentationParameters());
      intersectionEstimationParameters = reaMessager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionParameters, new IntersectionEstimationParameters());
      polygonizerParameters = reaMessager.createInput(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerParameters, new PolygonizerParameters());

      reaMessager.registerTopicListener(REAModuleAPI.RequestEntireModuleState, (messageContent) -> sendCurrentState());
   }

   private void sendCurrentState()
   {
      reaMessager.submitMessage(REAModuleAPI.OcTreePlanarRegionSegmentationEnable, enableSegmentation.get());
      reaMessager.submitMessage(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerEnable, enablePolygonizer.get());
      reaMessager.submitMessage(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionEnable, enableIntersectionCalulator.get());

      reaMessager.submitMessage(REAModuleAPI.OcTreePlanarRegionSegmentationParameters, planarRegionSegmentationParameters.get());
      reaMessager.submitMessage(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionParameters, intersectionEstimationParameters.get());
      reaMessager.submitMessage(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerParameters, polygonizerParameters.get());
   }

   public void loadConfiguration(FilePropertyHelper filePropertyHelper)
   {
      Boolean enableFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.OcTreeEnable);
      if (enableFile != null)
         isOcTreeEnabled.set(enableFile);
      Boolean enableSegmentationFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.OcTreePlanarRegionSegmentationEnable);
      if (enableSegmentationFile != null)
         enableSegmentation.set(enableSegmentationFile);
      Boolean enablePolygonizerFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerEnable);
      if (enablePolygonizerFile != null)
         enablePolygonizer.set(enablePolygonizerFile);
      Boolean enableIntersectionCalulatorFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionEnable);
      if (enableIntersectionCalulatorFile != null)
         enableIntersectionCalulator.set(enableIntersectionCalulatorFile);

      String planarRegionSegmentationParametersFile = filePropertyHelper.loadProperty(REAModuleAPI.OcTreePlanarRegionSegmentationParameters);
      if (planarRegionSegmentationParametersFile != null)
         planarRegionSegmentationParameters.set(PlanarRegionSegmentationParameters.parse(planarRegionSegmentationParametersFile));

      String polygonizerParametersFile = filePropertyHelper.loadProperty(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerParameters);
      if (polygonizerParametersFile != null)
         polygonizerParameters.set(PolygonizerParameters.parse(polygonizerParametersFile));

      String intersectionEstimationParametersFile = filePropertyHelper.loadProperty(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionParameters);
      if (intersectionEstimationParametersFile != null)
         intersectionEstimationParameters.set(IntersectionEstimationParameters.parse(intersectionEstimationParametersFile));
   }

   public void saveConfiguration(FilePropertyHelper filePropertyHelper)
   {
      filePropertyHelper.saveProperty(REAModuleAPI.OcTreePlanarRegionSegmentationEnable, enableSegmentation.get());
      filePropertyHelper.saveProperty(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerEnable, enablePolygonizer.get());
      filePropertyHelper.saveProperty(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionEnable, enableIntersectionCalulator.get());

      filePropertyHelper.saveProperty(REAModuleAPI.OcTreePlanarRegionSegmentationParameters, planarRegionSegmentationParameters.get().toString());
      filePropertyHelper.saveProperty(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerParameters, polygonizerParameters.get().toString());
      filePropertyHelper.saveProperty(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionParameters, intersectionEstimationParameters.get().toString());
   }

   public void update()
   {
      if (!isOcTreeEnabled.get())
         return;

      intersectionCalculator.clear();

      if (clearSegmentation.get())
      {
         planarRegionCalculator.clear();
         return;
      }

      if (!enableSegmentation.get())
      {
         planarRegionCalculator.removeDeadNodes();
         return;
      }

      planarRegionCalculator.setBoundingBox(octree.getBoundingBox());
      planarRegionCalculator.setParameters(planarRegionSegmentationParameters.get());
      intersectionCalculator.setParameters(intersectionEstimationParameters.get());

      timeReporter.run(() -> planarRegionCalculator.compute(octree.getRoot()), segmentationTimeReport);

      if (enablePolygonizer.get())
         timeReporter.run(this::updatePolygons, segmentationTimeReport);

      if (enableIntersectionCalulator.get())
         timeReporter.run(() -> intersectionCalculator.compute(planarRegionCalculator.getOcTreeNodePlanarRegions()), intersectionsTimeReport);
   }

   public void clearOcTree()
   {
      intersectionCalculator.clear();
      planarRegionCalculator.clear();
   }

   private void updatePolygons()
   {
      PolygonizerParameters parameters = polygonizerParameters.get();
      List<OcTreeNodePlanarRegion> planarRegions = planarRegionCalculator.getOcTreeNodePlanarRegions();

      concaveHulls = PlanarRegionPolygonizer.computeConcaveHulls(planarRegions, parameters);
      convexPolygons = PlanarRegionPolygonizer.computeConvexDecomposition(concaveHulls, parameters);
   }

   @Override
   public List<OcTreeNodePlanarRegion> getOcTreePlanarRegions()
   {
      return planarRegionCalculator.getOcTreeNodePlanarRegions();
   }

   @Override
   public boolean hasPolygonizedOcTreeNodePlanarRegions()
   {
      return concaveHulls != null;
   }

   @Override
   public PlanarRegionConcaveHull getPlanarRegionConcaveHull(OcTreeNodePlanarRegion planarRegion)
   {
      return concaveHulls == null ? null : concaveHulls.get(planarRegion);
   }

   @Override
   public PlanarRegionConvexPolygons getPlanarRegionConvexPolygons(OcTreeNodePlanarRegion planarRegion)
   {
      return convexPolygons == null ? null : convexPolygons.get(planarRegion);
   }

   @Override
   public int getNumberOfPlaneIntersections()
   {
      return intersectionCalculator.getNumberOfIntersections();
   }

   @Override
   public LineSegment3d getIntersection(int index)
   {
      return intersectionCalculator.getIntersection(index);
   }
}
