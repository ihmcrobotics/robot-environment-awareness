package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import javafx.geometry.Point3D;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Mesh;
import javafx.scene.transform.Rotate;
import javafx.util.Pair;
import us.ihmc.javaFXToolkit.shapes.MultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette2D;
import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.octoMap.iterators.OcTreeIterable;
import us.ihmc.octoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.geometry.IntersectionPlaneBoxCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.OcTreeNodePlanarRegion;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionConcaveHull;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionConvexPolygons;
import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class REAOcTreeGraphicsBuilder
{
   private static final Color DEFAULT_COLOR = Color.DARKCYAN;
   private static final Color OCTREE_BBX_COLOR = new Color(Color.DARKGREY.getRed(), Color.DARKGREY.getGreen(), Color.DARKGREY.getBlue(), 0.0);

   private final NormalOcTree octree;
   private final RegionFeaturesProvider regionFeaturesProvider;

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> clear;
   private final AtomicReference<Integer> treeDepthForDisplay;

   public enum ColoringType
   {
      DEFAULT, NORMAL, HAS_CENTER, REGION
   };

   private final AtomicReference<ColoringType> coloringType;

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

   private final REAMessager outputMessager;

   public REAOcTreeGraphicsBuilder(NormalOcTree octree, RegionFeaturesProvider regionFeaturesProvider, REAMessageManager inputManager, REAMessager outputMessager)
   {
      this.octree = octree;
      this.regionFeaturesProvider = regionFeaturesProvider;
      this.outputMessager = outputMessager;
      enable = inputManager.createInput(REAModuleAPI.OcTreeEnable);
      clear = inputManager.createInput(REAModuleAPI.OcTreeClear);

      treeDepthForDisplay = inputManager.createInput(REAModuleAPI.OcTreeGraphicsDepth);
      coloringType = inputManager.createInput(REAModuleAPI.OcTreeGraphicsColoringMode);

      showOcTreeNodes = inputManager.createInput(REAModuleAPI.OcTreeGraphicsShowOcTreeNodes);
      showEstimatedSurfaces = inputManager.createInput(REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces);
      hidePlanarRegionNodes = inputManager.createInput(REAModuleAPI.OcTreeGraphicsHidePlanarRegionNodes);
      showOcTreeBoundingBox = inputManager.createInput(REAModuleAPI.OcTreeGraphicsBoundingBoxShow);
      useOcTreeBoundingBox = inputManager.createInput(REAModuleAPI.OcTreeGraphicsBoundingBoxEnable);

      normalBasedColorPalette1D.setHueBased(0.9, 0.8);
      normalVariationBasedColorPalette1D.setBrightnessBased(0.0, 0.0);
      occupiedMeshBuilder = new MultiColorMeshBuilder(normalBasedColorPalette1D);
      TextureColorPalette2D regionColorPalette1D = new TextureColorPalette2D();
      regionColorPalette1D.setHueBrightnessBased(0.9);
      polygonsMeshBuilder = new MultiColorMeshBuilder(regionColorPalette1D);
   }

   private final RecyclingArrayList<Point3d> plane = new RecyclingArrayList<>(GenericTypeBuilder.createBuilderWithEmptyConstructor(Point3d.class));
   private final IntersectionPlaneBoxCalculator intersectionPlaneBoxCalculator = new IntersectionPlaneBoxCalculator();

   public void update()
   {
      if (shoudClear())
      {
         outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsOccupiedMesh, new Pair<Mesh, Material>(null, null)));
         outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsPlanarPolygonMesh, new Pair<Mesh, Material>(null, null)));
         return;
      }

      if (!isEnabled() && !processPropertyChange.get())
         return;

      processPropertyChange.set(false);

      occupiedMeshBuilder.clear();
      polygonsMeshBuilder.clear();

      addCellsToMeshBuilders(occupiedMeshBuilder);
      addPolygonsToMeshBuilders(polygonsMeshBuilder);

      if (Thread.interrupted())
         return;

      Material occupiedLeafMaterial = getOccupiedMeshMaterial();
      Pair<Mesh, Material> occupiedMeshAndMaterial = new Pair<Mesh, Material>(occupiedMeshBuilder.generateMesh(), occupiedLeafMaterial);

      Pair<Mesh, Material> planarRegionPolygonsMesh = new Pair<>(polygonsMeshBuilder.generateMesh(), polygonsMeshBuilder.generateMaterial());

      outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsOccupiedMesh, occupiedMeshAndMaterial));
      outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsPlanarPolygonMesh, planarRegionPolygonsMesh));

      handleOcTreeBoundingBox();
   }

   private void addPolygonsToMeshBuilders(MultiColorMeshBuilder polygonsMeshBuilder)
   {
      for (int i = 0; i < regionFeaturesProvider.getNumberOfPlaneIntersections(); i++)
      {
         LineSegment3d intersection = regionFeaturesProvider.getIntersection(i);
         Point3d start = intersection.getPointA();
         Point3d end = intersection.getPointB();
         double lineWidth = 0.025;
         Color color = Color.BLACK;
         polygonsMeshBuilder.addLine(start, end, lineWidth, color);
      }

      for (OcTreeNodePlanarRegion ocTreeNodePlanarRegion : regionFeaturesProvider.getOcTreePlanarRegions())
      {
         PlanarRegionConcaveHull planarRegionConcaveHull = regionFeaturesProvider.getPlanarRegionConcaveHull(ocTreeNodePlanarRegion);
         if (planarRegionConcaveHull == null)
            continue;

         double lineWidth = 0.01;
         int regionId = ocTreeNodePlanarRegion.getId();
         Color regionColor = getRegionColor(regionId);
         List<Point3d> concaveHullVerticesInWorld = planarRegionConcaveHull.getConcaveHullVerticesInWorld();
         polygonsMeshBuilder.addMultiLine(concaveHullVerticesInWorld, lineWidth, regionColor, true);

         PlanarRegionConvexPolygons planarRegionConvexPolygons = regionFeaturesProvider.getPlanarRegionConvexPolygons(ocTreeNodePlanarRegion);
         if (planarRegionConvexPolygons == null)
            continue;

         List<List<Point3d>> convexPolygonsVertices = planarRegionConvexPolygons.getConvexPolygonsVerticesInWorld();
         int numberOfConvexPolygons = convexPolygonsVertices.size();
         for (int j = 0; j < numberOfConvexPolygons; j++)
         {
            List<Point3d> convexPolygonVertices = convexPolygonsVertices.get(j);
            regionColor = Color.hsb(regionColor.getHue(), 0.9, 0.5 + 0.5 * ((double) j / (double) numberOfConvexPolygons));
            polygonsMeshBuilder.addPolyon(convexPolygonVertices, regionColor);
         }
      }
   }

   private void addCellsToMeshBuilders(MultiColorMeshBuilder occupiedMeshBuilder)
   {
      if (!isShowingOcTreeNodes())
         return;

      int currentDepth = getCurrentTreeDepthForDisplay();
      Set<NormalOcTreeNode> nodes = getNodes(currentDepth);

      for (OcTreeNodePlanarRegion ocTreeNodePlanarRegion : regionFeaturesProvider.getOcTreePlanarRegions())
      {
         for (NormalOcTreeNode node : ocTreeNodePlanarRegion)
         {
            nodes.remove(node);

            if (isHidingPlanarRegionNodes())
               continue;

            Color color = getNodeColor(node, ocTreeNodePlanarRegion.getId());
            addNodeToMeshBuilder(node, color, occupiedMeshBuilder);
         }
      }

      for (NormalOcTreeNode node : nodes)
      {
         Color color = getNodeColor(node);
         addNodeToMeshBuilder(node, color, occupiedMeshBuilder);
      }
   }

   private void addNodeToMeshBuilder(NormalOcTreeNode node, Color color, MultiColorMeshBuilder meshBuilder)
   {
      Vector3d planeNormal = new Vector3d();
      Point3d pointOnPlane = new Point3d();
      double size = node.getSize();

      if (node.isNormalSet() && isShowingEstimatedSurfaces())
      {
         node.getNormal(planeNormal);

         if (node.isHitLocationSet())
            node.getHitLocation(pointOnPlane);
         else
            node.getCoordinate(pointOnPlane);

         intersectionPlaneBoxCalculator.setCube(size, node.getX(), node.getY(), node.getZ());
         intersectionPlaneBoxCalculator.setPlane(pointOnPlane, planeNormal);
         intersectionPlaneBoxCalculator.computeIntersections(plane);
         meshBuilder.addPolyon(plane, color);
      }
      else
      {
         meshBuilder.addCube(size, node.getX(), node.getY(), node.getZ(), color);
      }
   }

   private Set<NormalOcTreeNode> getNodes(int currentDepth)
   {
      OcTreeIterable<NormalOcTreeNode> iterable = OcTreeIteratorFactory.createLeafIteratable(octree.getRoot(), currentDepth);
      if (useOcTreeBoundingBox())
         iterable.setRule(OcTreeIteratorFactory.leavesInsideBoundingBoxOnly(octree.getBoundingBox()));

      Set<NormalOcTreeNode> nodes = new HashSet<>();
      iterable.forEach(nodes::add);
      return nodes;
   }

   private void handleOcTreeBoundingBox()
   {
      if (!isShowingBoundingBox())
      {
         outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsBoundingBoxMesh, (Box) null));
         return;
      }

      OcTreeBoundingBoxWithCenterAndYaw boundingBox = (OcTreeBoundingBoxWithCenterAndYaw) octree.getBoundingBox();
      boundingBox.getLocalSize(ocTreeBoundingBoxSize);
      boundingBox.getCenterCoordinate(ocTreeBoundingBoxCenter);

      double yaw = boundingBox.getYaw();

      Box ocTreeBoundingBoxGraphics = new Box();
      ocTreeBoundingBoxGraphics.setWidth(ocTreeBoundingBoxSize.getX());
      ocTreeBoundingBoxGraphics.setHeight(ocTreeBoundingBoxSize.getY());
      ocTreeBoundingBoxGraphics.setDepth(ocTreeBoundingBoxSize.getZ());
      ocTreeBoundingBoxGraphics.setTranslateX(ocTreeBoundingBoxCenter.getX());
      ocTreeBoundingBoxGraphics.setTranslateY(ocTreeBoundingBoxCenter.getY());
      ocTreeBoundingBoxGraphics.setTranslateZ(ocTreeBoundingBoxCenter.getZ());
      ocTreeBoundingBoxGraphics.setRotationAxis(ocTreeBoudingBoxRotationAxis);
      ocTreeBoundingBoxGraphics.setRotate(Math.toDegrees(yaw));

      ocTreeBoundingBoxGraphics.setMaterial(ocTreeBoundingBoxMaterial);
      outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsBoundingBoxMesh, ocTreeBoundingBoxGraphics));
   }

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

   private int getCurrentTreeDepthForDisplay()
   {
      return treeDepthForDisplay.get() == null ? octree.getTreeDepth() : treeDepthForDisplay.get();
   }

   private boolean isShowingBoundingBox()
   {
      boolean ret = showOcTreeBoundingBox.get() == null ? false : showOcTreeBoundingBox.get();
      ret &= octree.getBoundingBox() != null;
      return ret;
   }

   private boolean useOcTreeBoundingBox()
   {
      return useOcTreeBoundingBox.get() == null ? false : useOcTreeBoundingBox.get();
   }

   private ColoringType getCurrentColoringType()
   {
      return coloringType.get() == null ? ColoringType.DEFAULT : coloringType.get();
   }
}
