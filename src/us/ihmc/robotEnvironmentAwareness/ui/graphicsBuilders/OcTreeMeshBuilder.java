package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

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
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionNodeKeysMessage;
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

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> clear;
   private final AtomicReference<Integer> treeDepthForDisplay;

   public enum ColoringType
   {
      DEFAULT, NORMAL, HAS_CENTER, REGION
   }

   private final AtomicReference<ColoringType> coloringType;

   private final AtomicReference<Boolean> showOcTreeNodes;
   private final AtomicReference<Boolean> showEstimatedSurfaces;
   private final AtomicReference<Boolean> hidePlanarRegionNodes;

   private final JavaFXMultiColorMeshBuilder meshBuilder;

   private final Material defaultOccupiedMaterial = new PhongMaterial(DEFAULT_COLOR);

   private final TextureColorPalette1D normalBasedColorPalette1D = new TextureColorPalette1D();
   private final TextureColorPalette1D normalVariationBasedColorPalette1D = new TextureColorPalette1D();

   private final RecyclingArrayList<Point3d> plane = new RecyclingArrayList<>(GenericTypeBuilder.createBuilderWithEmptyConstructor(Point3d.class));
   private final IntersectionPlaneBoxCalculator intersectionPlaneBoxCalculator = new IntersectionPlaneBoxCalculator();

   private final AtomicReference<NormalOcTreeMessage> ocTreeState;
   private final AtomicReference<PlanarRegionNodeKeysMessage[]> planarRegionNodeKeysState;

   private final AtomicReference<Pair<Mesh, Material>> meshAndMaterialToRender = new AtomicReference<>(null);

   private final REAUIMessager uiMessager;

   public OcTreeMeshBuilder(REAUIMessager uiMessager)
   {
      this.uiMessager = uiMessager;
      enable = uiMessager.createInput(REAModuleAPI.OcTreeEnable, false);

      clear = uiMessager.createInput(REAModuleAPI.OcTreeClear, false);

      treeDepthForDisplay = uiMessager.createInput(REAModuleAPI.UIOcTreeDepth);
      coloringType = uiMessager.createInput(REAModuleAPI.UIOcTreeColoringMode, ColoringType.DEFAULT);

      showOcTreeNodes = uiMessager.createInput(REAModuleAPI.UIOcTreeShow, false);
      showEstimatedSurfaces = uiMessager.createInput(REAModuleAPI.UINormalEstimationShow, false);
      hidePlanarRegionNodes = uiMessager.createInput(REAModuleAPI.UIPlanarRegionHideNodes, false);

      ocTreeState = uiMessager.createInput(REAModuleAPI.OcTreeState);
      planarRegionNodeKeysState = uiMessager.createInput(REAModuleAPI.PlanarRegionsNodeState);

      normalBasedColorPalette1D.setHueBased(0.9, 0.8);
      normalVariationBasedColorPalette1D.setBrightnessBased(0.0, 0.0);
      meshBuilder = new JavaFXMultiColorMeshBuilder(normalBasedColorPalette1D);
      TextureColorPalette2D regionColorPalette1D = new TextureColorPalette2D();
      regionColorPalette1D.setHueBrightnessBased(0.9);
   }

   @Override
   public void run()
   {
      if (clear.getAndSet(false))
      {
         meshAndMaterialToRender.set(new Pair<>(null, null));
         return;
      }

      if (!enable.get())
         return;

      uiMessager.submitStateRequestToModule(REAModuleAPI.RequestOctree);
      uiMessager.submitStateRequestToModule(REAModuleAPI.RequestPlanarRegionsNodeKeys);

      meshBuilder.clear();

      NormalOcTreeMessage newMessage = ocTreeState.get();
      Map<OcTreeKey, Integer> nodeKeyToRegionIdMap = createNodeKeyToRegionIdMap(planarRegionNodeKeysState.getAndSet(null));

      if (newMessage == null || nodeKeyToRegionIdMap == null)
         return;

      UIOcTree octree = new UIOcTree(ocTreeState.getAndSet(null), nodeKeyToRegionIdMap);

      addCellsToMeshBuilders(meshBuilder, octree);

      Mesh mesh = meshBuilder.generateMesh();
      Material material = getMaterial();
      meshAndMaterialToRender.set(new Pair<>(mesh, material));
   }

   private void addCellsToMeshBuilders(JavaFXMultiColorMeshBuilder meshBuilder, UIOcTree octree)
   {
      if (!showOcTreeNodes.get())
         return;

      // FIXME use treeDepthForDisplay
      Iterable<UIOcTreeNode> iterable;

      if (treeDepthForDisplay.get() != null)
         iterable = OcTreeIteratorFactory.createLeafIteratable(octree.getRoot(), treeDepthForDisplay.get());
      else
         iterable = octree;

      for (UIOcTreeNode node : iterable)
      {
         // FIXME
         if (hidePlanarRegionNodes.get())
            continue;

         Color color = getNodeColor(node);
         double size = node.getSize();

         if (node.isNormalSet() && showEstimatedSurfaces.get())
         {
            meshBuilder.addMesh(createNormalBasedPlane(node), color);
         }
         else
         {
            meshBuilder.addCube(size, node.getX(), node.getY(), node.getZ(), color);
         }
      }
   }

   private Color getNodeColor(UIOcTreeNode node)
   {
      switch (coloringType.get())
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

   private Color getRegionColor(int regionId)
   {
      java.awt.Color awtColor = new java.awt.Color(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
   }

   private Material getMaterial()
   {
      if (!showOcTreeNodes.get())
         return null;

      switch (coloringType.get())
      {
      case DEFAULT:
         return defaultOccupiedMaterial;
      case REGION:
      case NORMAL:
      case HAS_CENTER:
         meshBuilder.changeColorPalette(normalBasedColorPalette1D);
         return meshBuilder.generateMaterial();
      default:
         throw new RuntimeException("Unhandled ColoringType value: " + coloringType.get());
      }
   }

   private MeshDataHolder createNormalBasedPlane(UIOcTreeNode node)
   {
      Vector3d planeNormal = new Vector3d();
      Point3d pointOnPlane = new Point3d();
      double size = node.getSize();

      node.getNormal(planeNormal);

      if (node.isHitLocationSet())
         node.getHitLocation(pointOnPlane);
      else
         node.getCoordinate(pointOnPlane);

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

   private Map<OcTreeKey, Integer> createNodeKeyToRegionIdMap(PlanarRegionNodeKeysMessage[] planarRegionNodeKeysMessages)
   {
      if (planarRegionNodeKeysMessages == null)
         return null;

      Map<OcTreeKey, Integer> nodeKeyToRegionIdMap = new HashMap<>();

      for (PlanarRegionNodeKeysMessage planarRegionNodeKeysMessage : planarRegionNodeKeysMessages)
         registerNodeKeysIntoMap(nodeKeyToRegionIdMap, planarRegionNodeKeysMessage);

      return nodeKeyToRegionIdMap;
   }

   private void registerNodeKeysIntoMap(Map<OcTreeKey, Integer> nodeKeyToRegionIdMap, PlanarRegionNodeKeysMessage planarRegionNodeKeysMessage)
   {
      for (int i = 0; i < planarRegionNodeKeysMessage.getNumberOfNodes(); i++)
      {
         OcTreeKey nodeKey = new OcTreeKey();
         planarRegionNodeKeysMessage.getNodeKey(i, nodeKey);
         nodeKeyToRegionIdMap.put(nodeKey, planarRegionNodeKeysMessage.getRegionId());
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