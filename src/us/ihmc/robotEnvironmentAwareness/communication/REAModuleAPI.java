package us.ihmc.robotEnvironmentAwareness.communication;

public class REAModuleAPI
{
   private static final String OcTree = "OcTree/";
   private static final String LIDAR = "LIDAR/";
   private static final String Scan = "Scan/";
   private static final String BoundingBox = "BoundingBox/";
   private static final String Graphics = "Graphics/";
   private static final String Input = "Input/";
   private static final String Output = "Output/";
   private static final String NormalEstimation = "NormalEstimation/";
   private static final String PlanarRegionSegmentation = "PlanarRegionSegmentation/";
   private static final String PlanarRegionFeatures = "PlanarRegionFeatures/";
   private static final String PlanarRegions = "PlanarRegions/";
   private static final String Intersection = "Intersection/";
   private static final String Polygonizer = "Polygonizer/";
   private static final String Buffer = "Buffer/";

   public static final String OcTreeEnable = Input + OcTree + "Enable";
   public static final String OcTreeClear = Input + OcTree + "Clear";
   public static final String OcTreeLIDARMinRange = Input + OcTree + LIDAR + "MinRange";
   public static final String OcTreeLIDARMaxRange = Input + OcTree + LIDAR + "MaxRange";
   public static final String OcTreeBoundingBoxEnable = Input + OcTree + BoundingBox + "Enable";
   public static final String OcTreeBoundingBoxParameters = Input + OcTree + BoundingBox + "Parameters";
   public static final String OcTreeNormalEstimationEnable = Input + OcTree + NormalEstimation + "Enable";
   public static final String OcTreeNormalEstimationClear = Input + OcTree + NormalEstimation + "Clear";
   public static final String OcTreeNormalEstimationParameters = Input + OcTree + NormalEstimation + "Parameters";
   public static final String OcTreeBufferSize = Input + OcTree + Buffer + "Size";

   public static final String OcTreeGraphicsBoundingBoxEnable = Input + OcTree + Graphics + BoundingBox + "Enable";
   public static final String OcTreeGraphicsDepth = Input + OcTree + Graphics + "Depth";
   public static final String OcTreeGraphicsColoringMode = Input + OcTree + Graphics + "ColoringMode";
   public static final String OcTreeGraphicsShowOcTreeNodes = Input + OcTree + Graphics + "ShowOcTreeNodes";
   public static final String OcTreeGraphicsShowEstimatedSurfaces = Input + OcTree + Graphics + "ShowEstimatedSurfaces";
   public static final String OcTreeGraphicsHidePlanarRegionNodes = Input + OcTree + Graphics + "HidePlanarRegionNodes";
   public static final String OcTreeGraphicsBoundingBoxShow = Input + OcTree + Graphics + BoundingBox + "Show";
   public static final String OcTreeGraphicsShowBuffer = Input + OcTree + Graphics + Buffer + "Show";
   public static final String OcTreeGraphicsShowInputScan = Input + OcTree + Graphics + "ShowInputScan";

   // TODO Review the names
   public static final String OcTreePlanarRegionSegmentationEnable = Input + OcTree + PlanarRegionSegmentation + "Enable";
   public static final String OcTreePlanarRegionSegmentationClear = Input + OcTree + PlanarRegionSegmentation + "Clear";
   public static final String OcTreePlanarRegionSegmentationParameters = Input + OcTree + PlanarRegionSegmentation + "Parameters";
   public static final String OcTreePlanarRegionFeaturesPolygonizerEnable = Input + OcTree + PlanarRegionFeatures + Polygonizer + "Enable";
   public static final String OcTreePlanarRegionFeaturesIntersectionEnable = Input + OcTree + PlanarRegionFeatures + Intersection + "Enable";
   public static final String OcTreePlanarRegionFeaturesIntersectionParameters = Input + OcTree + PlanarRegionFeatures + Intersection + "Parameters";
   public static final String OcTreePlanarRegionFeaturesPolygonizerParameters = Input + OcTree + PlanarRegionFeatures + Polygonizer + "Parameters";

   public static final String OcTreeHasCleared = Output + OcTree + "HasCleared";

   public static final String OcTreeGraphicsOccupiedMesh = Output + OcTree + Graphics + "OccupiedMesh";
   public static final String OcTreeGraphicsBufferMesh = Output + OcTree + Graphics + Buffer + "Mesh";
   public static final String OcTreeGraphicsInputScanMesh = Output + OcTree + Graphics + "InputScan";
   public static final String OcTreeGraphicsPlanarPolygonMesh = Output + OcTree + Graphics + "PlanarPolygonMesh";
   public static final String OcTreeGraphicsBoundingBoxMesh = Output + OcTree + Graphics + BoundingBox + "Mesh";

   public static final String BufferOctreeNodeSize = Output + OcTree + Graphics + "BufferOctreeNodeSize";

   public static final String ScanPointsCollection = Output + OcTree + Graphics + "ScanPointsCollection";

   public static final String LidarScanState = Output + LIDAR + Scan + "State";
   public static final String OcTreeState = Output + OcTree + "State";
   public static final String BufferState = Output + OcTree + Buffer + "State";
   public static final String PlanarRegionsState = Output + PlanarRegions + "State";
   public static final String BoundingBoxState = Output + BoundingBox + "State";

   public static final String RequestLidarScan = Input + LIDAR + Scan + "Request";
   public static final String RequestOctree = Input + OcTree + "Request";
   public static final String RequestBuffer = Input + OcTree + Buffer + "Request";
   public static final String RequestPlanarRegions = Output + PlanarRegions + "Request";
   public static final String RequestBoundingBox = Input + BoundingBox + "Request";
}
