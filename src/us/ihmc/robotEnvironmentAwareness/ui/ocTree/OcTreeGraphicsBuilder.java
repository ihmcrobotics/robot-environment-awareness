package us.ihmc.robotEnvironmentAwareness.ui.ocTree;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.javaFXToolkit.shapes.MeshBuilder;
import us.ihmc.javaFXToolkit.shapes.MultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.octoMap.boundingBox.OcTreeSimpleBoundingBox;
import us.ihmc.octoMap.iterators.LeafBoundingBoxIterable;
import us.ihmc.octoMap.iterators.LeafIterable;
import us.ihmc.octoMap.iterators.OcTreeSuperNode;
import us.ihmc.octoMap.node.NormalOcTreeNode;
import us.ihmc.octoMap.ocTree.implementations.NormalOcTree;
import us.ihmc.octoMap.tools.IntersectionPlaneBoxCalculator;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class OcTreeGraphicsBuilder
{
   private static final Color DEFAULT_COLOR = Color.DARKCYAN;
   private static final Color FREE_COLOR = new Color(Color.YELLOW.getRed(), Color.YELLOW.getGreen(), Color.YELLOW.getBlue(), 0.01);

   private final NormalOcTree octree;

   private final AtomicBoolean enable;
   private final AtomicBoolean clear = new AtomicBoolean(false);
   private final AtomicInteger treeDepthForDisplay;

   public enum ColoringType {DEFAULT, NORMAL, NORMAL_VARIATION, HAS_CENTER, REGION};

   private final AtomicReference<ColoringType> coloringType = new AtomicReference<>(ColoringType.REGION);

   private final AtomicBoolean showFreeSpace = new AtomicBoolean(false);
   private final AtomicBoolean showEstimatedSurfaces = new AtomicBoolean(true);

   private final AtomicBoolean processPropertyChange = new AtomicBoolean(false);

   private final MultiColorMeshBuilder occupiedMeshBuilder;
   private final MeshBuilder freeMeshBuilder = new MeshBuilder();

   private final AtomicReference<Pair<Mesh, Material>> newOccupiedMeshToRender = new AtomicReference<>(null);
   private final AtomicReference<Pair<Mesh, Material>> newFreeMeshToRender = new AtomicReference<>(null);
   private final Material defaultOccupiedMaterial = new PhongMaterial(DEFAULT_COLOR);
   private final Material defaultFreeMaterial = new PhongMaterial(FREE_COLOR);

   private final TextureColorPalette1D normalBasedColorPalette1D = new TextureColorPalette1D();
   private final TextureColorPalette1D normalVariationBasedColorPalette1D = new TextureColorPalette1D();

   private final AtomicBoolean useBoundingBox = new AtomicBoolean(false);
   private final Point3d boundingBoxMin = new Point3d(-0.0, -2.0, -1.0);
   private final Point3d boundingBoxMax = new Point3d(10.0, 2.0, 1.0);
   private final AtomicReference<OcTreeSimpleBoundingBox> atomicBoundingBox;

   private final LeafIterable<NormalOcTreeNode> leafIterable;
   private final LeafBoundingBoxIterable<NormalOcTreeNode> leafBoundingBoxIterable;

   public OcTreeGraphicsBuilder(NormalOcTree octree, boolean enableInitialValue)
   {
      this.octree = octree;
      enable = new AtomicBoolean(enableInitialValue);
      treeDepthForDisplay = new AtomicInteger(octree.getTreeDepth() - 0);

      normalBasedColorPalette1D.setHueBased(0.9, 0.8);
      normalVariationBasedColorPalette1D.setBrightnessBased(0.0, 0.0);
      occupiedMeshBuilder = new MultiColorMeshBuilder(normalBasedColorPalette1D);

      leafIterable = new LeafIterable<>(octree, true);
      leafBoundingBoxIterable = new LeafBoundingBoxIterable<>(octree, true);

      double resolution = octree.getResolution();
      int treeDepth = octree.getTreeDepth();
      atomicBoundingBox = new AtomicReference<>(new OcTreeSimpleBoundingBox(boundingBoxMin, boundingBoxMax, resolution, treeDepth));
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

      int currentDepth = treeDepthForDisplay.get();
      occupiedMeshBuilder.clear();
      freeMeshBuilder.clear();
      boolean showSurfaces = showEstimatedSurfaces.get();

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
         leafBoundingBoxIterable.setBoundingBox(boundingBoxMin, boundingBoxMax);
         iterable = leafBoundingBoxIterable;
      }

      Vector3d planeNormal = new Vector3d();
      Point3d pointOnPlane = new Point3d();

      for (OcTreeSuperNode<NormalOcTreeNode> superNode : iterable)
      {
         NormalOcTreeNode node = superNode.getNode();
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

      if (Thread.interrupted())
         return;

      Material occupiedLeafMaterial = getOccupiedMeshMaterial();
      Pair<Mesh, Material> meshAndMaterial = new Pair<Mesh, Material>(occupiedMeshBuilder.generateMesh(), occupiedLeafMaterial);
      newOccupiedMeshToRender.set(meshAndMaterial);

      Mesh freeLeafMesh = showFreeSpace.get() ? freeMeshBuilder.generateMesh() : null;
      newFreeMeshToRender.set(new Pair<Mesh, Material>(freeLeafMesh, defaultFreeMaterial));
   }

   public void updateBoundingBox()
   {
      if (atomicBoundingBox.get() == null)
         return;
      OcTreeSimpleBoundingBox newBoundingBox = atomicBoundingBox.get();
      newBoundingBox.update(octree.getResolution(), octree.getTreeDepth());
      newBoundingBox.getMinCoordinate(boundingBoxMin);
      newBoundingBox.getMaxCoordinate(boundingBoxMax);
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

   public Pair<Mesh, Material> pollOccupiedMesh()
   {
      return newOccupiedMeshToRender.getAndSet(null);
   }
   
   public Pair<Mesh, Material> pollFreeMesh()
   {
      return newFreeMeshToRender.getAndSet(null);
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

   private Color getNodeColor(OcTreeSuperNode<NormalOcTreeNode> superNode)
   {
      NormalOcTreeNode node = superNode.getNode();

      switch (coloringType.get())
      {
      case REGION:
         if (node.isPartOfRegion())
         {
            java.awt.Color awtColor = new java.awt.Color(node.getRegionId());
            return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
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

      case NORMAL_VARIATION:
         double neighborNormalMeanDifference = octree.computeNodeNeighborNormalDifference(superNode.getKey(), superNode.getDepth());
         if (!Double.isNaN(neighborNormalMeanDifference))
         {
            double brightness = neighborNormalMeanDifference;
            brightness = Math.min(1.0, Math.max(0.0, brightness));
            return Color.hsb(0.0, 0.0, brightness);
         }
         else
            return DEFAULT_COLOR;
      case DEFAULT:
      default:
         return DEFAULT_COLOR;
      }
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
      case NORMAL_VARIATION:
         occupiedMeshBuilder.changeColorPalette(normalVariationBasedColorPalette1D);
         return occupiedMeshBuilder.generateMaterial();
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

   public OcTreeSimpleBoundingBox getBoundingBox()
   {
      return atomicBoundingBox.get();
   }

   public void setBoundingBox(OcTreeSimpleBoundingBox boundingBox)
   {
      if (useBoundingBox.get())
         processPropertyChange.set(true);
      atomicBoundingBox.set(boundingBox);
   }
}
