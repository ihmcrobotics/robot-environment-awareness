package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import javafx.geometry.Point3D;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.transform.Rotate;
import javafx.util.Pair;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.graphics3DDescription.MeshDataHolder;
import us.ihmc.jOctoMap.iterators.OcTreeIterable;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.shapes.MultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette2D;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OctreeNodeData;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OctreeNodeMessage;
import us.ihmc.robotEnvironmentAwareness.geometry.IntersectionPlaneBoxCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.OcTreeNodePlanarRegion;
import us.ihmc.robotEnvironmentAwareness.updaters.RegionFeaturesProvider;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;

import javax.vecmath.*;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Created by adrien on 11/21/16.
 */
public class OcTreeMeshBuilder implements Runnable
{

   private static final Color DEFAULT_COLOR = Color.DARKCYAN;
   private static final Color OCTREE_BBX_COLOR = new Color(Color.DARKGREY.getRed(), Color.DARKGREY.getGreen(), Color.DARKGREY.getBlue(), 0.0);


   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> clear;
   private final AtomicReference<Integer> treeDepthForDisplay;

   public enum ColoringType
   {
      DEFAULT, NORMAL, HAS_CENTER, REGION
   }



   private final AtomicReference<REAOcTreeGraphicsBuilder.ColoringType> coloringType;

   private final AtomicReference<Boolean> showOcTreeNodes;
   private final AtomicReference<Boolean> showEstimatedSurfaces;
   private final AtomicReference<Boolean> hidePlanarRegionNodes;

   private final AtomicBoolean processPropertyChange = new AtomicBoolean(false);

   private final MultiColorMeshBuilder occupiedMeshBuilder;
   private final MultiColorMeshBuilder polygonsMeshBuilder;

   private final Vector3d ocTreeBoundingBoxSize = new Vector3d();
   private final Point3d ocTreeBoundingBoxCenter = new Point3d();
   private final Point3D ocTreeBoudingBoxRotationAxis = Rotate.Z_AXIS;

   private final Material ocTreeBoundingBoxMaterial = new PhongMaterial(OCTREE_BBX_COLOR);
   private final Material defaultOccupiedMaterial = new PhongMaterial(DEFAULT_COLOR);

   private final TextureColorPalette1D normalBasedColorPalette1D = new TextureColorPalette1D();
   private final TextureColorPalette1D normalVariationBasedColorPalette1D = new TextureColorPalette1D();

   private final AtomicReference<Boolean> showOcTreeBoundingBox;
   private final AtomicReference<Boolean> useOcTreeBoundingBox;

   private final AtomicReference<ArrayList<OctreeNodeData>> octreeNodeDataList = new AtomicReference<>(null);
   private final AtomicReference<Float> nodeDefaultSize = new AtomicReference<>(null);

   private OctreeMeshBuilderListener listener;
   private boolean hasClearedBufferGraphics = false;

   private final RecyclingArrayList<Point3d> plane = new RecyclingArrayList<>(GenericTypeBuilder.createBuilderWithEmptyConstructor(Point3d.class));
   private final IntersectionPlaneBoxCalculator intersectionPlaneBoxCalculator = new IntersectionPlaneBoxCalculator();


   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>();

   public interface OctreeMeshBuilderListener
   {
      void occupiedMeshAndMaterialChanged(Pair<Mesh, Material> meshMaterial);

      void planarRegionMeshAndMaterialChanged(Pair<Mesh, Material> meshMaterial);
   }

   public OcTreeMeshBuilder(REAMessager reaMessager, OctreeMeshBuilderListener octreeMeshBuilderListener)
   {
      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable);

      clear = reaMessager.createInput(REAModuleAPI.OcTreeClear);

      treeDepthForDisplay = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsDepth);
      coloringType = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsColoringMode);

      showOcTreeNodes = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsShowOcTreeNodes);
      showEstimatedSurfaces = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces);
      hidePlanarRegionNodes = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsHidePlanarRegionNodes);
      showOcTreeBoundingBox = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsBoundingBoxShow);
      useOcTreeBoundingBox = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsBoundingBoxEnable);

      normalBasedColorPalette1D.setHueBased(0.9, 0.8);
      normalVariationBasedColorPalette1D.setBrightnessBased(0.0, 0.0);
      occupiedMeshBuilder = new MultiColorMeshBuilder(normalBasedColorPalette1D);
      TextureColorPalette2D regionColorPalette1D = new TextureColorPalette2D();
      regionColorPalette1D.setHueBrightnessBased(0.9);
      polygonsMeshBuilder = new MultiColorMeshBuilder(regionColorPalette1D);

      setListener(octreeMeshBuilderListener);

      reaMessager.getPacketCommunicator().attachListener(OctreeNodeMessage.class, octreeNodeMessagePacketConsumer);
      reaMessager.getPacketCommunicator().attachListener(PlanarRegionsListMessage.class, planarRegionListMessagePacketConsumer);
   }

   public PacketConsumer<PlanarRegionsListMessage> planarRegionListMessagePacketConsumer = (PacketConsumer<PlanarRegionsListMessage>) packet ->
   {
      if (packet == null)
         return;

      planarRegionsList.set(PlanarRegionMessageConverter.convertToPlanarRegionsList(new PlanarRegionsListMessage())); // TODO figure out what's going on with the planar list and planar list message

      System.out.println("new PlanarRegionList received! ");
   };



   public PacketConsumer<OctreeNodeMessage> octreeNodeMessagePacketConsumer = (PacketConsumer<OctreeNodeMessage>) packet ->
   {

      if (packet == null || packet.getMessageID() != REAModuleAPI.OctreeMessageID)
         return;

      octreeNodeDataList.set(packet.getOctreeNodeDataList());
      nodeDefaultSize.set(packet.getDefaultNodeSize());
      System.out.println(" new OctreeMessageID received! ");
   };



   private void setListener(OctreeMeshBuilderListener listener)
   {
      if (listener == null)
         throw new IllegalArgumentException("OctreeMeshBuilderListener cannot be null");
      this.listener = listener;
   }

   @Override public void run()
   {
//      System.out.println("run");
      if (shoudClear())
      {
         //         reaMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsOccupiedMesh, new Pair<Mesh, Material>(null, null)));
         //         reaMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsPlanarPolygonMesh, new Pair<Mesh, Material>(null, null)));
         return;
      }

      if (!isEnabled() && !processPropertyChange.get())
         return;

      processPropertyChange.set(false);

      occupiedMeshBuilder.clear();
      polygonsMeshBuilder.clear();

      if(octreeNodeDataList.get() != null)
      {
         ArrayList<OctreeNodeData> data = octreeNodeDataList.getAndSet(null);

         addCellsToMeshBuilders(occupiedMeshBuilder, data);

//         addPolygonsToMeshBuilders(polygonsMeshBuilder);

         System.out.println("still runming ");
         if (Thread.interrupted())
            return;

         MeshDataHolder occupiedMeshDataHolder = occupiedMeshBuilder.generateMeshDataHolder();
         //      MeshDataHolder planarRegionMeshDataHolder = polygonsMeshBuilder.generateMeshDataHolder();

         Material occupiedLeafMaterial = getOccupiedMeshMaterial();
         //      Material planarRegionMaterial = polygonsMeshBuilder.generateMaterial();

         TriangleMesh occupiedMesh = JavaFXMeshDataInterpreter.interpretMeshData(occupiedMeshDataHolder);
         Pair<Mesh, Material> occupiedMeshAndMaterial = new Pair<Mesh, Material>(occupiedMesh, occupiedLeafMaterial);

         System.out.println(" calling listener ");
         listener.occupiedMeshAndMaterialChanged(occupiedMeshAndMaterial);

//         TriangleMesh planarRegionMesh = JavaFXMeshDataInterpreter.interpretMeshData(planarRegionMeshDataHolder);
//         Pair<Mesh, Material> planarRegionMeshAndMaterial = new Pair<Mesh, Material>(planarRegionMesh, planarRegionMaterial);
//         listener.planarRegionMeshAndMaterialChanged(planarRegionMeshAndMaterial);
      }
   }

   //   private Runnable createMeshCreationFutureTask()
   //   {
   //      MeshDataHolder occupiedMeshDataHolder = occupiedMeshBuilder.generateMeshDataHolder();
   //      MeshDataHolder planarRegionMeshDataHolder = polygonsMeshBuilder.generateMeshDataHolder();
   //
   //      Material occupiedLeafMaterial = getOccupiedMeshMaterial();
   //      Material planarRegionMaterial = polygonsMeshBuilder.generateMaterial();
   //
   //      return new Runnable()
   //      {
   //         @Override
   //         public void run()
   //         {
   //            processAndSubmitMesh(REAModuleAPI.OcTreeGraphicsOccupiedMesh, occupiedMeshDataHolder, occupiedLeafMaterial);
   //            processAndSubmitMesh(REAModuleAPI.OcTreeGraphicsPlanarPolygonMesh, planarRegionMeshDataHolder, planarRegionMaterial);
   //         }
   //      };
   //   }

//      private void addPolygonsToMeshBuilders(MultiColorMeshBuilder polygonsMeshBuilder)
//      {
//         for (int i = 0; i < regionFeaturesProvider.getNumberOfPlaneIntersections(); i++)
//         {
//            LineSegment3d intersection = regionFeaturesProvider.getIntersection(i);
//            Point3d start = intersection.getPointA();
//            Point3d end = intersection.getPointB();
//            double lineWidth = 0.025;
//            Color color = Color.BLACK;
//            polygonsMeshBuilder.addLine(start, end, lineWidth, color);
//         }
//
//         for (OcTreeNodePlanarRegion ocTreeNodePlanarRegion : regionFeaturesProvider.getOcTreePlanarRegions())
//         {
//            PlanarRegionConcaveHull planarRegionConcaveHull = regionFeaturesProvider.getPlanarRegionConcaveHull(ocTreeNodePlanarRegion);
//            if (planarRegionConcaveHull == null)
//               continue;
//
//            double lineWidth = 0.01;
//            int regionId = ocTreeNodePlanarRegion.getId();
//            Color regionColor = getRegionColor(regionId);
//            List<Point3d> concaveHullVerticesInWorld = planarRegionConcaveHull.getConcaveHullVerticesInWorld();
//            polygonsMeshBuilder.addMultiLine(concaveHullVerticesInWorld, lineWidth, regionColor, true);
//
//            PlanarRegionConvexPolygons planarRegionConvexPolygons = regionFeaturesProvider.getPlanarRegionConvexPolygons(ocTreeNodePlanarRegion);
//            if (planarRegionConvexPolygons == null)
//               continue;
//
//            List<List<Point3d>> convexPolygonsVertices = planarRegionConvexPolygons.getConvexPolygonsVerticesInWorld();
//            int numberOfConvexPolygons = convexPolygonsVertices.size();
//            for (int j = 0; j < numberOfConvexPolygons; j++)
//            {
//               List<Point3d> convexPolygonVertices = convexPolygonsVertices.get(j);
//               regionColor = Color.hsb(regionColor.getHue(), 0.9, 0.5 + 0.5 * ((double) j / (double) numberOfConvexPolygons));
//               polygonsMeshBuilder.addPolyon(convexPolygonVertices, regionColor);
//            }
//         }
//      }

   private void addCellsToMeshBuilders(MultiColorMeshBuilder occupiedMeshBuilder, ArrayList<OctreeNodeData> octreeNodes)
   {
      if (!isShowingOcTreeNodes())
         return;

      System.out.println(" -->  showing octree nodes ");
      for (OctreeNodeData node : octreeNodes)
      {
         if (isHidingPlanarRegionNodes())
            continue;

         Color color = getNodeColor(node.getNormalOcTreeNode(), node.getRegionId());
         addNodeToMeshBuilder(node.getNormalOcTreeNode(), color, occupiedMeshBuilder);
      }
   }

   private void addNodeToMeshBuilder(NormalOcTreeNode node, Color color, MultiColorMeshBuilder meshBuilder)
   {
      double size = node.getSize();

      if (node.isNormalSet() && isShowingEstimatedSurfaces())
      {
         meshBuilder.addMesh(createNormalBasedPlane(node), color);
      }
      else
      {
         meshBuilder.addCube(size, node.getX(), node.getY(), node.getZ(), color);
      }
   }

   private Set<NormalOcTreeNode> getNodes(NormalOcTree octree, int currentDepth)
   {
      OcTreeIterable<NormalOcTreeNode> iterable = OcTreeIteratorFactory.createLeafIteratable(octree.getRoot(), currentDepth);
      if (useOcTreeBoundingBox())
         iterable.setRule(OcTreeIteratorFactory.leavesInsideBoundingBoxOnly(octree.getBoundingBox()));

      Set<NormalOcTreeNode> nodes = new HashSet<>();
      iterable.forEach(nodes::add);
      return nodes;
   }

   //   private void handleOcTreeBoundingBox() // TODO
   //   {
   //
   ////      if (!isShowingBoundingBox())
   ////      {
   ////         //reaMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsBoundingBoxMesh, (Box) null));
   ////         return;
   ////      }
   //
   //      OcTreeBoundingBoxWithCenterAndYaw boundingBox = (OcTreeBoundingBoxWithCenterAndYaw) octree.getBoundingBox();
   //      boundingBox.getLocalSize(ocTreeBoundingBoxSize);
   //      boundingBox.getCenterCoordinate(ocTreeBoundingBoxCenter);
   //
   //      double yaw = boundingBox.getYaw();
   //
   //      Box ocTreeBoundingBoxGraphics = new Box();
   //      ocTreeBoundingBoxGraphics.setWidth(ocTreeBoundingBoxSize.getX());
   //      ocTreeBoundingBoxGraphics.setHeight(ocTreeBoundingBoxSize.getY());
   //      ocTreeBoundingBoxGraphics.setDepth(ocTreeBoundingBoxSize.getZ());
   //      ocTreeBoundingBoxGraphics.setTranslateX(ocTreeBoundingBoxCenter.getX());
   //      ocTreeBoundingBoxGraphics.setTranslateY(ocTreeBoundingBoxCenter.getY());
   //      ocTreeBoundingBoxGraphics.setTranslateZ(ocTreeBoundingBoxCenter.getZ());
   //      ocTreeBoundingBoxGraphics.setRotationAxis(ocTreeBoudingBoxRotationAxis);
   //      ocTreeBoundingBoxGraphics.setRotate(Math.toDegrees(yaw));
   //
   //      ocTreeBoundingBoxGraphics.setMaterial(ocTreeBoundingBoxMaterial);
   //      //      reaMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsBoundingBoxMesh, ocTreeBoundingBoxGraphics));
   //   }

   private Color getNodeColor(NormalOcTreeNode node)
   {
      return getNodeColor(node, OcTreeNodePlanarRegion.NO_REGION_ID);
   }

   private Color getNodeColor(NormalOcTreeNode node, int regionId)
   {
      switch (getCurrentColoringType())
      {
      case REGION:
         if (regionId != OcTreeNodePlanarRegion.NO_REGION_ID)
            return getRegionColor(regionId);
         else
            return DEFAULT_COLOR;
      case HAS_CENTER:
         return node.isHitLocationSet() ? Color.DARKGREEN : Color.RED;
      case NORMAL:
         if (node.isNormalSet())
         {
            Vector3d normal = new Vector3d();
            node.getNormal(normal);
            Vector3d zUp = new Vector3d(0.0, 0.0, 1.0);
            normal.normalize();
            double angle = Math.abs(zUp.dot(normal));
            double hue = 120.0 * angle;
            return Color.hsb(hue, 1.0, 1.0);
         }
         else
            return DEFAULT_COLOR;
      case DEFAULT:
      default:
         return DEFAULT_COLOR;
      }
   }

   private Color getRegionColor(int regionId)
   {
      java.awt.Color awtColor = new java.awt.Color(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
   }

   private Material getOccupiedMeshMaterial()
   {
      if (!isShowingOcTreeNodes())
         return null;

      switch (getCurrentColoringType())
      {
      case DEFAULT:
         return defaultOccupiedMaterial;
      case REGION:
      case NORMAL:
      case HAS_CENTER:
         occupiedMeshBuilder.changeColorPalette(normalBasedColorPalette1D);
         return occupiedMeshBuilder.generateMaterial();
      default:
         throw new RuntimeException("Unhandled ColoringType value: " + getCurrentColoringType());
      }
   }

   private MeshDataHolder createNormalBasedPlane(NormalOcTreeNode node)
   {
      Vector3d planeNormal = new Vector3d();
      Point3d pointOnPlane = new Point3d();
//      double size = node.getSize();
      double size = nodeDefaultSize.get(); // TODO What to do?!! node.getSize returns NaN

      node.getNormal(planeNormal);

      if (node.isHitLocationSet())
         node.getHitLocation(pointOnPlane);
      else
         node.getCoordinate(pointOnPlane);

      intersectionPlaneBoxCalculator.setCube(size, node.getHitLocationX(), node.getHitLocationY(), node.getHitLocationZ()); // TODO What to do?!! node.getX() - getZ() return NaN
      intersectionPlaneBoxCalculator.setPlane(pointOnPlane, planeNormal);
      intersectionPlaneBoxCalculator.computeIntersections(plane);

      if (plane.size() < 3)
         return null;

      int numberOfTriangles = plane.size() - 2;
      int[] triangleIndices = new int[3 * numberOfTriangles];
      int index = 0;
      for (int j = 2; j < plane.size(); j++)
      {
         triangleIndices[index++] = 0;
         triangleIndices[index++] = j - 1;
         triangleIndices[index++] = j;
      }

      Point3f[] vertices = new Point3f[plane.size()];
      TexCoord2f[] texCoords = new TexCoord2f[plane.size()];
      Vector3f[] normals = new Vector3f[plane.size()];

      for (int i = 0; i < plane.size(); i++)
      {
         vertices[i] = new Point3f(plane.get(i));
         texCoords[i] = new TexCoord2f(); // No need for real coordinates, the MultiColorMeshBuilder creates new ones.
         normals[i] = new Vector3f(planeNormal);
      }

      return new MeshDataHolder(vertices, texCoords, triangleIndices, normals);
   }

   private boolean isEnabled()
   {
      return enable.get() == null ? false : enable.get();
   }

   private boolean shoudClear()
   {
      return clear.get() == null ? false : clear.getAndSet(null);
   }

   private boolean isShowingOcTreeNodes()
   {
      return showOcTreeNodes.get() == null ? false : showOcTreeNodes.get();
   }

   private boolean isHidingPlanarRegionNodes()
   {
      return hidePlanarRegionNodes.get() == null ? false : hidePlanarRegionNodes.get();
   }

   private boolean isShowingEstimatedSurfaces()
   {
      return showEstimatedSurfaces.get() == null ? false : showEstimatedSurfaces.get();
   }

   //   private boolean isShowingBoundingBox()
   //   {
   //      boolean ret = showOcTreeBoundingBox.get() == null ? false : showOcTreeBoundingBox.get();
   //      ret &= octree.getBoundingBox() != null;
   //      return ret;
   //   }

   private boolean useOcTreeBoundingBox()
   {
      return useOcTreeBoundingBox.get() == null ? false : useOcTreeBoundingBox.get();
   }

   private REAOcTreeGraphicsBuilder.ColoringType getCurrentColoringType()
   {
      return coloringType.get() == null ? REAOcTreeGraphicsBuilder.ColoringType.DEFAULT : coloringType.get();
   }

}
