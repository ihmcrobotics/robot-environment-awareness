package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.List;
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
import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.octoMap.iterators.LeafBoundingBoxIterable;
import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.ocTree.implementations.NormalOcTree;
import us.ihmc.octoMap.planarRegions.PlanarRegion;
import us.ihmc.octoMap.tools.IntersectionPlaneBoxCalculator;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionIntersectionCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.time.TimeTools;

public class REAOcTreeGraphicsBuilder
{
   private static final Color DEFAULT_COLOR = Color.DARKCYAN;
   private static final Color OCTREE_BBX_COLOR = new Color(Color.DARKGREY.getRed(), Color.DARKGREY.getGreen(), Color.DARKGREY.getBlue(), 0.0);

   private final NormalOcTree octree;

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
   private final AtomicReference<Boolean> showPlanarRegions;

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

   private final LeafBoundingBoxIterable<NormalOcTreeNode> leafBoundingBoxIterable;

   private final PlanarRegionIntersectionCalculator intersectionCalculator = new PlanarRegionIntersectionCalculator();
   private final PlanarRegionPolygonizer planarRegionPolygonizer = new PlanarRegionPolygonizer();
   private final REAMessager outputMessager;

   public REAOcTreeGraphicsBuilder(NormalOcTree octree, REAMessageManager inputManager, REAMessager outputMessager)
   {
      this.octree = octree;
      this.outputMessager = outputMessager;
      enable = inputManager.createInput(REAModuleAPI.OcTreeEnable, Boolean.class);
      clear = inputManager.createInput(REAModuleAPI.OcTreeClear, Boolean.class);

      treeDepthForDisplay = inputManager.createInput(REAModuleAPI.OcTreeGraphicsDepth, Integer.class);
      coloringType = inputManager.createInput(REAModuleAPI.OcTreeGraphicsColoringMode, ColoringType.class);

      showOcTreeNodes = inputManager.createInput(REAModuleAPI.OcTreeGraphicsShowOcTreeNodes, Boolean.class);
      showEstimatedSurfaces = inputManager.createInput(REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces, Boolean.class);
      hidePlanarRegionNodes = inputManager.createInput(REAModuleAPI.OcTreeGraphicsHidePlanarRegionNodes, Boolean.class);
      showPlanarRegions = inputManager.createInput(REAModuleAPI.OcTreeGraphicsShowPlanarRegions, Boolean.class);
      showOcTreeBoundingBox = inputManager.createInput(REAModuleAPI.OcTreeGraphicsBoundingBoxShow, Boolean.class);
      useOcTreeBoundingBox = inputManager.createInput(REAModuleAPI.OcTreeGraphicsBoundingBoxEnable, Boolean.class);

      normalBasedColorPalette1D.setHueBased(0.9, 0.8);
      normalVariationBasedColorPalette1D.setBrightnessBased(0.0, 0.0);
      occupiedMeshBuilder = new MultiColorMeshBuilder(normalBasedColorPalette1D);
      TextureColorPalette2D regionColorPalette1D = new TextureColorPalette2D();
      regionColorPalette1D.setHueBrightnessBased(0.9);
      polygonsMeshBuilder = new MultiColorMeshBuilder(regionColorPalette1D);

      leafBoundingBoxIterable = new LeafBoundingBoxIterable<>(octree, true);
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

      Mesh planarRegionPolygonMesh = isShowingPlanarRegions() ? polygonsMeshBuilder.generateMesh() : null;
      Pair<Mesh, Material> planarRegionPolygonsMesh = new Pair<>(planarRegionPolygonMesh, polygonsMeshBuilder.generateMaterial());

      outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsOccupiedMesh, occupiedMeshAndMaterial));
      outputMessager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsPlanarPolygonMesh, planarRegionPolygonsMesh));

      handleOcTreeBoundingBox();
   }

   private void addPolygonsToMeshBuilders(MultiColorMeshBuilder polygonsMeshBuilder)
   {
      if (!isShowingPlanarRegions())
         return;

      long startTime = System.nanoTime();

      intersectionCalculator.compute(octree.getPlanarRegions());

      for (int i = 0; i < intersectionCalculator.getNumberOfIntersections(); i++)
      {
         LineSegment3d intersection = intersectionCalculator.getIntersection(i);
         Point3d start = intersection.getPointA();
         Point3d end = intersection.getPointB();
         double lineWidth = 0.025;
         Color color = Color.BLACK;
         polygonsMeshBuilder.addLineMesh(start, end, lineWidth, color);
      }

      for (int i = 0; i < octree.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion planarRegion = octree.getPlanarRegion(i);
         if (planarRegion.getNumberOfNodes() < 10)
            continue;
         //         planarRegion.printPointsToFile();
         planarRegionPolygonizer.compute(planarRegion);
         List<Point3d> regionConacveHullVertices = planarRegionPolygonizer.getConcaveHullVertices();
         if (regionConacveHullVertices.size() < 10)
            continue;

         double lineWidth = 0.01;
         int regionId = planarRegion.getId();
         Color regionColor = getRegionColor(regionId);
         polygonsMeshBuilder.addMultiLineMesh(regionConacveHullVertices, lineWidth, regionColor, true);

         for (int j = 0; j < planarRegionPolygonizer.getNumberOfConvexPolygons(); j++)
         {
            List<Point3d> convexPolygonVertices = planarRegionPolygonizer.getConvexPolygonVertices(j);
            regionColor = Color.hsb(regionColor.getHue(), 0.9, 0.5 + 0.5 * ((double) j / (double) planarRegionPolygonizer.getNumberOfConvexPolygons()));
            polygonsMeshBuilder.addPolyon(convexPolygonVertices, regionColor);
         }
      }

      long endTime = System.nanoTime();
      System.out.println("Processing concave hulls took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));
   }

   private void addCellsToMeshBuilders(MultiColorMeshBuilder occupiedMeshBuilder)
   {
      if (!isShowingOcTreeNodes())
         return;

      int currentDepth = getCurrentTreeDepthForDisplay();
      boolean showSurfaces = isShowingEstimatedSurfaces();

      Iterable<OcTreeSuperNode<NormalOcTreeNode>> iterable = updateAndGetIterable(currentDepth);

      Vector3d planeNormal = new Vector3d();
      Point3d pointOnPlane = new Point3d();

      for (OcTreeSuperNode<NormalOcTreeNode> superNode : iterable)
      {
         NormalOcTreeNode node = superNode.getNode();

         if (isHidingPlanarRegionNodes() && node.isPartOfRegion())
            continue;

         if (octree.isNodeOccupied(node))
         {
            double size = superNode.getSize();
            Point3d nodeCenter = superNode.getCoordinate();
            Color color = getNodeColor(superNode);
            if (node.isNormalSet() && showSurfaces)
            {
               node.getNormal(planeNormal);
               if (node.isCenterSet())
                  node.getCenter(pointOnPlane);
               else
                  pointOnPlane.set(nodeCenter);
               intersectionPlaneBoxCalculator.setCube(size, nodeCenter);
               intersectionPlaneBoxCalculator.setPlane(pointOnPlane, planeNormal);
               intersectionPlaneBoxCalculator.computeIntersections(plane);
               occupiedMeshBuilder.addPolyon(plane, color);
            }
            else
               occupiedMeshBuilder.addCubeMesh(size, nodeCenter, color);
         }
      }
   }

   private Iterable<OcTreeSuperNode<NormalOcTreeNode>> updateAndGetIterable(int currentDepth)
   {
      Iterable<OcTreeSuperNode<NormalOcTreeNode>> iterable;

      leafBoundingBoxIterable.setMaxDepth(currentDepth);
      if (useOcTreeBoundingBox())
      {
         OcTreeBoundingBoxInterface boundingBox = octree.getBoundingBox();
         leafBoundingBoxIterable.setBoundingBox(boundingBox);
      }
      else
      {
         leafBoundingBoxIterable.setBoundingBox(null);
      }
      iterable = leafBoundingBoxIterable;
      return iterable;
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

   private Color getNodeColor(OcTreeSuperNode<NormalOcTreeNode> superNode)
   {
      NormalOcTreeNode node = superNode.getNode();

      switch (getCurrentColoringType())
      {
      case REGION:
         if (node.isPartOfRegion())
         {
            int regionId = node.getRegionId();
            return getRegionColor(regionId);
         }
         return DEFAULT_COLOR;
      case HAS_CENTER:
         return node.isCenterSet() ? Color.DARKGREEN : Color.RED;
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

   private boolean isShowingPlanarRegions()
   {
      return showPlanarRegions.get() == null ? false : showPlanarRegions.get();
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
