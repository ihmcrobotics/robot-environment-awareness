package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.API;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.Category;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.CategoryTheme;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.TopicTheme;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.TypedTopicTheme;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoxMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.LineSegment3dMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionNodeKeysMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.ColoringType;

public class REAModuleAPI
{
   private static final APIFactory apiFactory = new APIFactory();

   private static final CategoryTheme Module = apiFactory.createCategoryTheme("Module");
   private static final CategoryTheme UI = apiFactory.createCategoryTheme("UserInterface");

   private static final CategoryTheme OcTree = apiFactory.createCategoryTheme("OcTree");
   private static final CategoryTheme Lidar = apiFactory.createCategoryTheme("Lidar");
   private static final CategoryTheme BoundingBox = apiFactory.createCategoryTheme("BoundingBox");
   private static final CategoryTheme NormalEstimation = apiFactory.createCategoryTheme("NormalEstimation");
   private static final CategoryTheme PlanarRegions = apiFactory.createCategoryTheme("PlanarRegions");
   private static final CategoryTheme Segmentation = apiFactory.createCategoryTheme("Segmentation");
   private static final CategoryTheme Intersection = apiFactory.createCategoryTheme("Intersection");
   private static final CategoryTheme Polygonizer = apiFactory.createCategoryTheme("Polygonizer");
   private static final CategoryTheme Buffer = apiFactory.createCategoryTheme("Buffer");
   private static final CategoryTheme Range = apiFactory.createCategoryTheme("Range");
   private static final CategoryTheme Node = apiFactory.createCategoryTheme("Node");
   private static final CategoryTheme Request = apiFactory.createCategoryTheme("Request");

   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Clear = apiFactory.createTypedTopicTheme("Clear");
   private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");
   private static final TypedTopicTheme<Boolean> Hide = apiFactory.createTypedTopicTheme("Hide");
   private static final TypedTopicTheme<Boolean> Save = apiFactory.createTypedTopicTheme("Save");
   private static final TypedTopicTheme<Integer> Size = apiFactory.createTypedTopicTheme("Size");
   private static final TypedTopicTheme<Integer> Depth = apiFactory.createTypedTopicTheme("Depth");

   private static final TopicTheme Parameters = apiFactory.createTopicTheme("Parameters");
   private static final TopicTheme Min = apiFactory.createTopicTheme("Min");
   private static final TopicTheme Max = apiFactory.createTopicTheme("Max");
   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");
   private static final TopicTheme Color = apiFactory.createTopicTheme("Color");

   private static final Category Root = apiFactory.getRootCategory(apiFactory.createCategoryTheme("REA"));

   private static final Category ModuleCategory = Root.child(Module);
   private static final Category OcTreeCategory = ModuleCategory.child(OcTree);
   private static final Category PlanarRegionsCategory = ModuleCategory.child(PlanarRegions);


   public static final Topic<Boolean> OcTreeEnable = OcTreeCategory.topic(Enable);
   public static final Topic<Boolean> OcTreeClear = OcTreeCategory.topic(Clear);
   public static final Topic<Integer> OcTreeBufferSize = OcTreeCategory.child(Buffer).topic(Size);
   public static final Topic<Boolean> OcTreeBoundingBoxEnable = OcTreeCategory.child(BoundingBox).topic(Enable);
   public static final Topic<BoundingBoxParametersMessage> OcTreeBoundingBoxParameters = OcTreeCategory.child(BoundingBox).topic(Parameters);

   public static final Topic<Boolean> NormalEstimationEnable = ModuleCategory.child(NormalEstimation).topic(Enable);
   public static final Topic<Boolean> NormalEstimationClear = ModuleCategory.child(NormalEstimation).topic(Clear);
   public static final Topic<NormalEstimationParameters> NormalEstimationParameters = ModuleCategory.child(NormalEstimation).topic(Parameters);

   public static final Topic<Double> LidarMinRange = ModuleCategory.child(Lidar).child(Range).topic(Min);
   public static final Topic<Double> LidarMaxRange = ModuleCategory.child(Lidar).child(Range).topic(Max);
   
   public static final Topic<Boolean> PlanarRegionsSegmentationEnable = PlanarRegionsCategory.child(Segmentation).topic(Enable);
   public static final Topic<Boolean> PlanarRegionsSegmentationClear = PlanarRegionsCategory.child(Segmentation).topic(Clear);
   public static final Topic<PlanarRegionSegmentationParameters> PlanarRegionsSegmentationParameters = PlanarRegionsCategory.child(Segmentation).topic(Parameters);
   public static final Topic<Boolean> PlanarRegionsPolygonizerEnable = PlanarRegionsCategory.child(Polygonizer).topic(Enable);
   public static final Topic<PolygonizerParameters> PlanarRegionsPolygonizerParameters = PlanarRegionsCategory.child(Polygonizer).topic(Parameters);
   public static final Topic<Boolean> PlanarRegionsIntersectionEnable = PlanarRegionsCategory.child(Intersection).topic(Enable);
   public static final Topic<IntersectionEstimationParameters> PlanarRegionsIntersectionParameters = PlanarRegionsCategory.child(Intersection).topic(Parameters);

   public static final Topic<Boolean> UIOcTreeShow = Root.child(UI).child(OcTree).topic(Show);
   public static final Topic<Integer> UIOcTreeDepth = Root.child(UI).child(OcTree).topic(Depth);
   public static final Topic<ColoringType> UIOcTreeColoringMode = Root.child(UI).child(OcTree).topic(Color);
   public static final Topic<Boolean> UINormalEstimationShow = Root.child(UI).child(NormalEstimation).topic(Show);
   public static final Topic<Boolean> UIPlanarRegionHideNodes = Root.child(UI).child(PlanarRegions).child(Node).topic(Hide);
   public static final Topic<Boolean> UIOcTreeBoundingBoxShow = Root.child(UI).child(OcTree).child(BoundingBox).topic(Show);
   public static final Topic<Boolean> UIOcTreeShowBuffer = Root.child(UI).child(OcTree).child(Buffer).topic(Show);
   public static final Topic<Boolean> UILidarScanShow = Root.child(UI).child(Lidar).topic(Show);
   public static final Topic<Boolean> UILidarScanClear = Root.child(UI).child(Lidar).topic(Clear);
   public static final Topic<Integer> UILidarScanSize = Root.child(UI).child(Lidar).topic(Size);

   public static final Topic<LidarScanMessage> LidarScanState = ModuleCategory.child(Lidar).topic(Data);
   public static final Topic<NormalOcTreeMessage> OcTreeState = OcTreeCategory.topic(Data);
   public static final Topic<NormalOcTreeMessage> OcTreeBufferState = OcTreeCategory.child(Buffer).topic(Data);
   public static final Topic<PlanarRegionsListMessage> PlanarRegionsState = PlanarRegionsCategory.topic(Data);
   public static final Topic<PlanarRegionNodeKeysMessage[]> PlanarRegionsNodeState = PlanarRegionsCategory.child(Node).topic(Data);
   public static final Topic<LineSegment3dMessage[]> PlanarRegionsIntersectionState = PlanarRegionsCategory.child(Intersection).topic(Data);
   public static final Topic<BoxMessage> OcTreeBoundingBoxState = OcTreeCategory.child(BoundingBox).topic(Data);

   public static final Topic<Boolean> RequestEntireModuleState = ModuleCategory.child(Request).topic(Data);
   public static final Topic<Boolean> RequestLidarScan = ModuleCategory.child(Lidar).child(Request).topic(Data);
   public static final Topic<Boolean> RequestOctree = OcTreeCategory.child(Request).topic(Data);
   public static final Topic<Boolean> RequestBuffer = OcTreeCategory.child(Buffer).child(Request).topic(Data);
   public static final Topic<Boolean> RequestPlanarRegions = PlanarRegionsCategory.child(Request).topic(Data);
   public static final Topic<Boolean> RequestPlanarRegionsIntersections = PlanarRegionsCategory.child(Intersection).child(Request).topic(Data);
   public static final Topic<Boolean> RequestPlanarRegionsNodeKeys = PlanarRegionsCategory.child(Request).child(Node).topic(Data);
   public static final Topic<Boolean> RequestBoundingBox = OcTreeCategory.child(BoundingBox).child(Request).topic(Data);

   public static final Topic<Boolean> SaveMainUpdaterConfiguration = OcTreeCategory.topic(Save);
   public static final Topic<Boolean> SaveBufferConfiguration = OcTreeCategory.child(Buffer).topic(Save);
   public static final Topic<Boolean> SaveRegionUpdaterConfiguration = PlanarRegionsCategory.topic(Save);

   public static final API API = apiFactory.getAPIAndCloseFactory();
}