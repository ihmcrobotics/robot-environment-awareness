package us.ihmc.robotEnvironmentAwareness.updaters;

public class REAModuleAPI
{
   private static final String OcTree = "OcTree/";
   private static final String LIDAR = "LIDAR/";
   private static final String BoundingBox = "BoundingBox/";
   private static final String Graphics = "Graphics/";
   private static final String Input = "Input/";
   private static final String Output = "Output/";
   
   public static final String OcTreeEnable = Input + OcTree + "Enable";
   public static final String OcTreeClear = Input + OcTree + "Clear";
   public static final String OcTreeLIDARMinRange = Input + OcTree + LIDAR + "MinRange";
   public static final String OcTreeLIDARMaxRange = Input + OcTree + LIDAR + "MaxRange";
   public static final String OcTreeBoundingBoxEnable = Input + OcTree + BoundingBox + "Enable";
   public static final String OcTreeBoundingBoxParameters = Input + OcTree + BoundingBox + "Parameters";

   public static final String OcTreeGraphicsBoundingBoxEnable = Input + OcTree + Graphics + BoundingBox + "Enable";
   public static final String OcTreeGraphicsDepth = Input + OcTree + Graphics + "Depth";
   public static final String OcTreeGraphicsColoringMode = Input + OcTree + Graphics + "ColoringMode";
   public static final String OcTreeGraphicsShowOcTreeNodes = Input + OcTree + Graphics + "ShowOcTreeNodes";
   public static final String OcTreeGraphicsShowEstimatedSurfaces = Input + OcTree + Graphics + "ShowEstimatedSurfaces";
   public static final String OcTreeGraphicsHidePlanarRegionNodes = Input + OcTree + Graphics + "HidePlanarRegionNodes";
   public static final String OcTreeGraphicsShowPlanarRegions = Input + OcTree + Graphics + "ShowPlanarRegions";
   public static final String OcTreeGraphicsBoundingBoxShow = Input + OcTree + Graphics + BoundingBox + "Show";


   public static final String OcTreeHasCleared = Output + OcTree + "HasCleared";

   public static final String OcTreeGraphicsOccupiedMesh = Output + OcTree + Graphics + "OccupiedMesh";
   public static final String OcTreeGraphicsPlanarPolygonMesh = Output + OcTree + Graphics + "PlanarPolygonMesh";
   public static final String OcTreeGraphicsBoundingBoxMesh = Output + OcTree + Graphics + BoundingBox + "Mesh";
}
