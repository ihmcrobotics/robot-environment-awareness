package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import javafx.beans.property.Property;
import javafx.beans.value.ObservableValue;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.graphics3DDescription.MeshDataHolder;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette2D;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotEnvironmentAwareness.geometry.IntersectionPlaneBoxCalculator;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;

/**
 * Created by adrien on 11/21/16.
 */
public class OcTreeMeshBuilder implements Runnable
{
   private static final Color DEFAULT_COLOR = Color.DARKCYAN;

   public enum ColoringType
   {
      DEFAULT, NORMAL, HAS_CENTER, REGION
   }

   public enum DisplayType
   {
      HIDE, CELL, PLANE, HIT_LOCATION
   }

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> clear;

   private final Property<ColoringType> coloringType;
   private final Property<DisplayType> displayType;
   private final Property<Integer> treeDepthForDisplay;
   private final Property<Boolean> hidePlanarRegionNodes;

   private final JavaFXMultiColorMeshBuilder meshBuilder;

   private final Material defaultOccupiedMaterial = new PhongMaterial(DEFAULT_COLOR);

   private final TextureColorPalette1D normalBasedColorPalette1D = new TextureColorPalette1D();
   private final TextureColorPalette1D normalVariationBasedColorPalette1D = new TextureColorPalette1D();

   private final RecyclingArrayList<Point3d> plane = new RecyclingArrayList<>(GenericTypeBuilder.createBuilderWithEmptyConstructor(Point3d.class));
   private final IntersectionPlaneBoxCalculator intersectionPlaneBoxCalculator = new IntersectionPlaneBoxCalculator();

   private final AtomicReference<NormalOcTreeMessage> ocTreeState;
   private final AtomicReference<PlanarRegionSegmentationMessage[]> planarRegionSegmentationState;

   private final AtomicBoolean processChange = new AtomicBoolean(false);

   private final AtomicReference<Pair<Mesh, Material>> meshAndMaterialToRender = new AtomicReference<>(null);

   private final REAUIMessager uiMessager;
   private final AtomicReference<UIOcTree> uiOcTree = new AtomicReference<UIOcTree>(null);

   public OcTreeMeshBuilder(REAUIMessager uiMessager)
   {
      this.uiMessager = uiMessager;
      enable = uiMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      clear = uiMessager.createInput(REAModuleAPI.OcTreeClear, false);

      treeDepthForDisplay = uiMessager.createPropertyInput(REAModuleAPI.UIOcTreeDepth);
      treeDepthForDisplay.addListener(this::setProcessChange);
      coloringType = uiMessager.createPropertyInput(REAModuleAPI.UIOcTreeColoringMode, ColoringType.DEFAULT);
      coloringType.addListener(this::setProcessChange);

      displayType = uiMessager.createPropertyInput(REAModuleAPI.UIOcTreeDisplayType, DisplayType.PLANE);
      displayType.addListener(this::setProcessChange);
      hidePlanarRegionNodes = uiMessager.createPropertyInput(REAModuleAPI.UIPlanarRegionHideNodes, false);
      hidePlanarRegionNodes.addListener(this::setProcessChange);

      ocTreeState = uiMessager.createInput(REAModuleAPI.OcTreeState);
      planarRegionSegmentationState = uiMessager.createInput(REAModuleAPI.PlanarRegionsSegmentationState);

      normalBasedColorPalette1D.setHueBased(0.9, 0.8);
      normalVariationBasedColorPalette1D.setBrightnessBased(0.0, 0.0);
      meshBuilder = new JavaFXMultiColorMeshBuilder(normalBasedColorPalette1D);
      TextureColorPalette2D regionColorPalette1D = new TextureColorPalette2D();
      regionColorPalette1D.setHueBrightnessBased(0.9);
   }

   private <T> void setProcessChange(ObservableValue<? extends T> observableValue, T oldValue, T newValue)
   {
      try
      {
         processChange.set(!oldValue.equals(newValue));
      }
      catch (NullPointerException e)
      {
         processChange.set(oldValue != newValue);
      }
   }

   @Override
   public void run()
   {
      if (clear.getAndSet(false))
      {
         meshAndMaterialToRender.set(new Pair<>(null, null));
         return;
      }

      if (enable.get())
      {
         uiMessager.submitStateRequestToModule(REAModuleAPI.RequestOctree);
         uiMessager.submitStateRequestToModule(REAModuleAPI.RequestPlanarRegionSegmentation);


         NormalOcTreeMessage newMessage = ocTreeState.get();
         Map<OcTreeKey, Integer> nodeKeyToRegionIdMap = createNodeKeyToRegionIdMap(planarRegionSegmentationState.getAndSet(null));

         if (newMessage == null || nodeKeyToRegionIdMap == null)
            return;

         uiOcTree.set(new UIOcTree(ocTreeState.getAndSet(null), nodeKeyToRegionIdMap));
         buildUIOcTreeMesh();
      }
      else if (processChange.getAndSet(false) && uiOcTree.get() != null)
      {
         buildUIOcTreeMesh();
      }
   }

   private void buildUIOcTreeMesh()
   {
      meshBuilder.clear();

      addCellsToMeshBuilders(meshBuilder, uiOcTree.get());

      Mesh mesh = meshBuilder.generateMesh();
      Material material = getMaterial();
      meshAndMaterialToRender.set(new Pair<>(mesh, material));
   }

   private void addCellsToMeshBuilders(JavaFXMultiColorMeshBuilder meshBuilder, UIOcTree octree)
   {
      if (displayType.getValue() == DisplayType.HIDE)
         return;

      Iterable<UIOcTreeNode> iterable;

      if (treeDepthForDisplay.getValue() != null)
         iterable = OcTreeIteratorFactory.createLeafIteratable(octree.getRoot(), treeDepthForDisplay.getValue());
      else
         iterable = octree;

      for (UIOcTreeNode node : iterable)
      {
         if (node.isPartOfRegion() && hidePlanarRegionNodes.getValue())
            continue;

         Color color = getNodeColor(node);
         double size = node.getSize();

         switch (displayType.getValue())
         {
         case CELL:
            meshBuilder.addCube(size, node.getX(), node.getY(), node.getZ(), color);
            break;
         case PLANE:
            if (node.isNormalSet())
               meshBuilder.addMesh(createNormalBasedPlane(node), color);
            break;
         case HIT_LOCATION:
            if (node.isHitLocationSet())
            {
               Point3d hitLocation = new Point3d();
               node.getHitLocation(hitLocation);
               meshBuilder.addTetrahedron(0.0075, hitLocation, color);
            }
            break;
         default:
            throw new RuntimeException("Unexpected value for display type: " + displayType.getValue());
         }
      }
   }

   private Color getNodeColor(UIOcTreeNode node)
   {
      switch (coloringType.getValue())
      {
      case REGION:
         if (node.isPartOfRegion())
            return getRegionColor(node.getRegionId());
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

   public static Color getRegionColor(int regionId)
   {
      java.awt.Color awtColor = new java.awt.Color(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
   }

   private Material getMaterial()
   {
      if (displayType.getValue() == DisplayType.HIDE)
         return null;

      switch (coloringType.getValue())
      {
      case DEFAULT:
         return defaultOccupiedMaterial;
      case REGION:
      case NORMAL:
      case HAS_CENTER:
         meshBuilder.changeColorPalette(normalBasedColorPalette1D);
         return meshBuilder.generateMaterial();
      default:
         throw new RuntimeException("Unhandled ColoringType value: " + coloringType.getValue());
      }
   }

   private MeshDataHolder createNormalBasedPlane(UIOcTreeNode node)
   {
      if (!node.isNormalSet() || !node.isHitLocationSet())
         return null;

      Vector3d planeNormal = new Vector3d();
      Point3d pointOnPlane = new Point3d();
      double size = node.getSize();

      node.getNormal(planeNormal);
      node.getHitLocation(pointOnPlane);

      intersectionPlaneBoxCalculator.setCube(size, node.getX(), node.getY(), node.getZ());
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

   private Map<OcTreeKey, Integer> createNodeKeyToRegionIdMap(PlanarRegionSegmentationMessage[] planarRegionSegmentationMessages)
   {
      if (planarRegionSegmentationMessages == null)
         return null;

      Map<OcTreeKey, Integer> nodeKeyToRegionIdMap = new HashMap<>();

      for (PlanarRegionSegmentationMessage planarRegionSegmentationMessage : planarRegionSegmentationMessages)
         registerNodeKeysIntoMap(nodeKeyToRegionIdMap, planarRegionSegmentationMessage);

      return nodeKeyToRegionIdMap;
   }

   private void registerNodeKeysIntoMap(Map<OcTreeKey, Integer> nodeKeyToRegionIdMap, PlanarRegionSegmentationMessage planarRegionSegmentationMessage)
   {
      for (int i = 0; i < planarRegionSegmentationMessage.getNumberOfNodes(); i++)
      {
         OcTreeKey nodeKey = new OcTreeKey();
         planarRegionSegmentationMessage.getNodeKey(i, nodeKey);
         nodeKeyToRegionIdMap.put(nodeKey, planarRegionSegmentationMessage.getRegionId());
      }
   }

   public boolean hasNewMeshAndMaterial()
   {
      return meshAndMaterialToRender.get() != null;
   }

   public Pair<Mesh, Material> pollMeshAndMaterial()
   {
      return meshAndMaterialToRender.getAndSet(null);
   }
}
