package us.ihmc.robotEnvironmentAwareness.ui.obsolete.ocTree;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
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
import us.ihmc.javaFXToolkit.shapes.MeshBuilder;
import us.ihmc.javaFXToolkit.shapes.MultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette2D;
import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.octoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.octoMap.iterators.LeafBoundingBoxIterable;
import us.ihmc.octoMap.iterators.LeafIterable;
import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.ocTree.implementations.NormalOcTree;
import us.ihmc.octoMap.planarRegions.PlanarRegion;
import us.ihmc.octoMap.tools.IntersectionPlaneBoxCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionIntersectionCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.time.TimeTools;

public class OcTreeGraphicsBuilder
{
   private static final Color DEFAULT_COLOR = Color.DARKCYAN;
   private static final Color FREE_COLOR = new Color(Color.YELLOW.getRed(), Color.YELLOW.getGreen(), Color.YELLOW.getBlue(), 0.01);
   private static final Color OCTREE_BBX_COLOR = new Color(Color.DARKGREY.getRed(), Color.DARKGREY.getGreen(), Color.DARKGREY.getBlue(), 0.0);

   private final NormalOcTree octree;

   private final AtomicBoolean enable;
   private final AtomicBoolean clear = new AtomicBoolean(false);
   private final AtomicInteger treeDepthForDisplay;

   public enum ColoringType {HIDE_NODES, DEFAULT, NORMAL, HAS_CENTER, REGION};

   private final AtomicReference<ColoringType> coloringType = new AtomicReference<>(ColoringType.REGION);

   private final AtomicBoolean showFreeSpace = new AtomicBoolean(false);
   private final AtomicBoolean showEstimatedSurfaces = new AtomicBoolean(true);
   private final AtomicBoolean hidePlanarRegionNodes = new AtomicBoolean(false);
   private final AtomicBoolean showPlanarRegions = new AtomicBoolean(false);

   private final AtomicBoolean processPropertyChange = new AtomicBoolean(false);

   private final MultiColorMeshBuilder occupiedMeshBuilder;
   private final MeshBuilder freeMeshBuilder = new MeshBuilder();
   private final MultiColorMeshBuilder polygonsMeshBuilder;

   private final Vector3d ocTreeBoundingBoxSize = new Vector3d();
   private final Point3d ocTreeBoundingBoxCenter = new Point3d();
   private final Point3D ocTreeBoudingBoxRotationAxis = Rotate.Z_AXIS;

   private final Material ocTreeBoundingBoxMaterial = new PhongMaterial(OCTREE_BBX_COLOR);
   private final AtomicReference<Box> newOcTreeBoudingBoxToRender = new AtomicReference<>(null);

   private final AtomicReference<Pair<Mesh, Material>> newOccupiedMeshToRender = new AtomicReference<>(null);
   private final AtomicReference<Pair<Mesh, Material>> newFreeMeshToRender = new AtomicReference<>(null);
   private final AtomicReference<Pair<Mesh, Material>> newPlanarRegionPolygonMeshToRender = new AtomicReference<>(null);
   private final Material defaultOccupiedMaterial = new PhongMaterial(DEFAULT_COLOR);
   private final Material defaultFreeMaterial = new PhongMaterial(FREE_COLOR);

   private final TextureColorPalette1D normalBasedColorPalette1D = new TextureColorPalette1D();
   private final TextureColorPalette1D normalVariationBasedColorPalette1D = new TextureColorPalette1D();

   private final AtomicBoolean useBoundingBox = new AtomicBoolean(false);
   private final Point3d boundingBoxMin = new Point3d(-0.0, -2.0, -1.1);
   private final Point3d boundingBoxMax = new Point3d(10.0, 2.0, 1.0);
   private final AtomicReference<OcTreeBoundingBoxWithCenterAndYaw> atomicBoundingBox;
   private final AtomicBoolean showOcTreeBoundingBox = new AtomicBoolean(false);

   private final LeafIterable<NormalOcTreeNode> leafIterable;
   private final LeafBoundingBoxIterable<NormalOcTreeNode> leafBoundingBoxIterable;

   private final PlanarRegionIntersectionCalculator intersectionCalculator = new PlanarRegionIntersectionCalculator();
   private final PlanarRegionPolygonizer planarRegionPolygonizer = new PlanarRegionPolygonizer();

   public OcTreeGraphicsBuilder(NormalOcTree octree, boolean enableInitialValue)
   {
      this.octree = octree;
      enable = new AtomicBoolean(enableInitialValue);
      treeDepthForDisplay = new AtomicInteger(octree.getTreeDepth() - 0);

      normalBasedColorPalette1D.setHueBased(0.9, 0.8);
      normalVariationBasedColorPalette1D.setBrightnessBased(0.0, 0.0);
      occupiedMeshBuilder = new MultiColorMeshBuilder(normalBasedColorPalette1D);
      TextureColorPalette2D regionColorPalette1D = new TextureColorPalette2D();
      regionColorPalette1D.setHueBrightnessBased(0.9);
      polygonsMeshBuilder = new MultiColorMeshBuilder(regionColorPalette1D);

      leafIterable = new LeafIterable<>(octree, true);
      leafBoundingBoxIterable = new LeafBoundingBoxIterable<>(octree, true);

      double resolution = octree.getResolution();
      int treeDepth = octree.getTreeDepth();
      atomicBoundingBox = new AtomicReference<>(new OcTreeBoundingBoxWithCenterAndYaw(boundingBoxMin, boundingBoxMax, resolution, treeDepth));
   }

   private final RecyclingArrayList<Point3d> plane = new RecyclingArrayList<>(GenericTypeBuilder.createBuilderWithEmptyConstructor(Point3d.class));
   private final IntersectionPlaneBoxCalculator intersectionPlaneBoxCalculator = new IntersectionPlaneBoxCalculator();

   public void update()
   {
      if (clear.getAndSet(false))
      {
         newOccupiedMeshToRender.set(null);
         newFreeMeshToRender.set(null);
         return;
      }

      if (!enable.get() && !processPropertyChange.get())
         return;

      processPropertyChange.set(false);

      occupiedMeshBuilder.clear();
      freeMeshBuilder.clear();
      polygonsMeshBuilder.clear();

      addCellsToMeshBuilders(occupiedMeshBuilder, freeMeshBuilder);
      addPolygonsToMeshBuilders(polygonsMeshBuilder);

      if (Thread.interrupted())
         return;

      Material occupiedLeafMaterial = getOccupiedMeshMaterial();
      Pair<Mesh, Material> meshAndMaterial = new Pair<Mesh, Material>(occupiedMeshBuilder.generateMesh(), occupiedLeafMaterial);
      newOccupiedMeshToRender.set(meshAndMaterial);

      Mesh freeLeafMesh = showFreeSpace.get() ? freeMeshBuilder.generateMesh() : null;
      newFreeMeshToRender.set(new Pair<Mesh, Material>(freeLeafMesh, defaultFreeMaterial));

      Mesh planarRegionPolygonMesh = showPlanarRegions.get() ? polygonsMeshBuilder.generateMesh() : null;
      newPlanarRegionPolygonMeshToRender.set(new Pair<>(planarRegionPolygonMesh, polygonsMeshBuilder.generateMaterial()));

      handleOcTreeBoundingBox();
   }

   private void addPolygonsToMeshBuilders(MultiColorMeshBuilder polygonsMeshBuilder)
   {
      if (!showPlanarRegions.get())
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

   private void addCellsToMeshBuilders(MultiColorMeshBuilder occupiedMeshBuilder, MeshBuilder freeMeshBuilder)
   {
      if (coloringType.get() == ColoringType.HIDE_NODES)
         return;

      int currentDepth = treeDepthForDisplay.get();
      boolean showSurfaces = showEstimatedSurfaces.get();

      Iterable<OcTreeSuperNode<NormalOcTreeNode>> iterable = updateAndGetIterable(currentDepth);

      Vector3d planeNormal = new Vector3d();
      Point3d pointOnPlane = new Point3d();

      for (OcTreeSuperNode<NormalOcTreeNode> superNode : iterable)
      {
         NormalOcTreeNode node = superNode.getNode();

         if (hidePlanarRegionNodes.get() && node.isPartOfRegion())
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
         else if (showFreeSpace.get())
         {
            double size = superNode.getSize();
            Point3d position = superNode.getCoordinate();
            freeMeshBuilder.addCubeMesh(size, position);
         }
      }
   }

   private Iterable<OcTreeSuperNode<NormalOcTreeNode>> updateAndGetIterable(int currentDepth)
   {
      Iterable<OcTreeSuperNode<NormalOcTreeNode>> iterable;

      if (!useBoundingBox.get())
      {
         leafIterable.setMaxDepth(currentDepth);
         iterable = leafIterable;
      }
      else
      {
         updateBoundingBox();
         leafBoundingBoxIterable.setMaxDepth(currentDepth);
         OcTreeBoundingBoxInterface boundingBox = atomicBoundingBox.getAndSet(null);
         if (boundingBox != null)
            leafBoundingBoxIterable.setBoundingBox(boundingBox);
         iterable = leafBoundingBoxIterable;
      }
      return iterable;
   }

   private void handleOcTreeBoundingBox()
   {
      OcTreeBoundingBoxWithCenterAndYaw boundingBox = (OcTreeBoundingBoxWithCenterAndYaw) octree.getBoundingBox();
      if (!showOcTreeBoundingBox.get() || boundingBox == null)
      {
         newOcTreeBoudingBoxToRender.set(null);
         return;
      }

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
      newOcTreeBoudingBoxToRender.set(ocTreeBoundingBoxGraphics);
   }

   public void updateBoundingBox()
   {
      if (atomicBoundingBox.get() == null)
         return;
      OcTreeBoundingBoxWithCenterAndYaw newBoundingBox = atomicBoundingBox.get();
      newBoundingBox.update(octree.getResolution(), octree.getTreeDepth());
   }

   public void setEnable(boolean enable)
   {
      this.enable.set(enable);
   }

   public void clear()
   {
      clear.set(true);
   }

   public boolean hasProcessedClear()
   {
      return !clear.get();
   }

   public boolean hasNewOccupiedMeshToRender()
   {
      return newOccupiedMeshToRender.get() != null;
   }

   public boolean hasNewFreeMeshToRender()
   {
      return newFreeMeshToRender.get() != null;
   }

   public boolean hasNewOcTreeBoundingBoxToRender()
   {
      return newOcTreeBoudingBoxToRender.get() != null;
   }

   public boolean hasNewPlanarRegionPolygonMeshToRender()
   {
      return newPlanarRegionPolygonMeshToRender.get() != null;
   }

   public Pair<Mesh, Material> pollOccupiedMesh()
   {
      return newOccupiedMeshToRender.getAndSet(null);
   }
   
   public Pair<Mesh, Material> pollFreeMesh()
   {
      return newFreeMeshToRender.getAndSet(null);
   }

   public Box pollOcTreeBoundingBoxGraphics()
   {
      return newOcTreeBoudingBoxToRender.get();
   }

   public Pair<Mesh, Material> pollPlanarRegionPolygonMesh()
   {
      return newPlanarRegionPolygonMeshToRender.getAndSet(null);
   }

   public void setTreeDepthForDisplay(int newDepth)
   {
      if (newDepth <= octree.getTreeDepth() && newDepth >= 0 && newDepth != treeDepthForDisplay.get())
      {
         processPropertyChange.set(true);
         treeDepthForDisplay.set(newDepth);
      }
   }

   public int getCurrentTreeDepthForDisplay()
   {
      return treeDepthForDisplay.get();
   }

   public void showFreeSpace(boolean show)
   {
      if (show != showFreeSpace.get())
      {
         processPropertyChange.set(true);
         showFreeSpace.set(show);
      }
   }

   public boolean isShowingFreeSpace()
   {
      return showFreeSpace.get();
   }

   public void setColoringType(ColoringType type)
   {
      if (type != coloringType.get())
      {
         processPropertyChange.set(true);
         coloringType.set(type);
      }
   }

   public ColoringType getColoringType()
   {
      return coloringType.get();
   }

   public void showEstimatedSurfaces(boolean show)
   {
      if (show != showEstimatedSurfaces.get())
      {
         processPropertyChange.set(true);
         showEstimatedSurfaces.set(show);
      }
   }

   public boolean isShowingEstimatedSurfaces()
   {
      return showEstimatedSurfaces.get();
   }

   public void showPlanarRegions(boolean show)
   {
      if (show != showPlanarRegions.get())
      {
         processPropertyChange.set(true);
         showPlanarRegions.set(show);
      }
   }

   public boolean isShowingPlanarRegions()
   {
      return showPlanarRegions.get();
   }

   public void hidePlanarRegionNodes(boolean show)
   {
      if (show != hidePlanarRegionNodes.get())
      {
         processPropertyChange.set(true);
         hidePlanarRegionNodes.set(show);
      }
   }

   public boolean isHidingPlanarRegionNodes()
   {
      return hidePlanarRegionNodes.get();
   }

   public void showOcTreeBoundingBox(boolean show)
   {
      if (show != showOcTreeBoundingBox.get())
      {
         processPropertyChange.set(true);
         showOcTreeBoundingBox.set(show);
      }
   }

   public boolean isShowingOcTreeBoundingBox()
   {
      return showOcTreeBoundingBox.get();
   }

   private Color getNodeColor(OcTreeSuperNode<NormalOcTreeNode> superNode)
   {
      NormalOcTreeNode node = superNode.getNode();

      switch (coloringType.get())
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
      case HIDE_NODES:
         return null;
      case DEFAULT:
      default:
         return DEFAULT_COLOR;
      }
   }

   public Color getRegionColor(int regionId)
   {
      java.awt.Color awtColor = new java.awt.Color(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
   }

   private Material getOccupiedMeshMaterial()
   {
      switch (coloringType.get())
      {
      case DEFAULT:
         return defaultOccupiedMaterial;
      case REGION:
      case NORMAL:
      case HAS_CENTER:
         occupiedMeshBuilder.changeColorPalette(normalBasedColorPalette1D);
         return occupiedMeshBuilder.generateMaterial();
      case HIDE_NODES:
         return null;
      default:
         throw new RuntimeException("Unhandled ColoringType value: " + coloringType.get());
      }
   }

   public boolean isBoundingBoxEnabled()
   {
      return useBoundingBox.get();
   }

   public void enableBoundingBox(boolean enable)
   {
      processPropertyChange.set(true);
      useBoundingBox.set(enable);
   }

   public OcTreeBoundingBoxWithCenterAndYaw getBoundingBox()
   {
      return atomicBoundingBox.get();
   }

   public void setBoundingBox(OcTreeBoundingBoxWithCenterAndYaw boundingBox)
   {
      if (useBoundingBox.get())
         processPropertyChange.set(true);
      atomicBoundingBox.set(boundingBox);
   }
}
