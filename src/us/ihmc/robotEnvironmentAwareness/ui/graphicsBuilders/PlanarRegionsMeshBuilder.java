package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.concurrent.atomic.AtomicReference;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette2D;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class PlanarRegionsMeshBuilder implements Runnable
{
   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> clear;

   private final JavaFXMultiColorMeshBuilder polygonsMeshBuilder;

   private final AtomicReference<PlanarRegionsListMessage> planarRegionsListMessage;

   private final AtomicReference<Pair<Mesh, Material>> meshAndMaterialToRender = new AtomicReference<>(null);

   private final REAMessager reaMessager;

   public PlanarRegionsMeshBuilder(REAMessager reaMessager)
   {
      this.reaMessager = reaMessager;
      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      clear = reaMessager.createInput(REAModuleAPI.OcTreeClear, false);

      planarRegionsListMessage = reaMessager.createInput(REAModuleAPI.PlanarRegionsState);

      TextureColorPalette2D regionColorPalette1D = new TextureColorPalette2D();
      regionColorPalette1D.setHueBrightnessBased(0.9);
      polygonsMeshBuilder = new JavaFXMultiColorMeshBuilder(regionColorPalette1D);
   }

   @Override
   public void run()
   {
      PlanarRegionsListMessage newMessage = planarRegionsListMessage.get();

      if (clear.getAndSet(false))
      {
         meshAndMaterialToRender.set(new Pair<>(null, null));
         return;
      }

      if (!enable.get())
         return;

      reaMessager.submitMessage(REAModuleAPI.RequestPlanarRegions, true);

      if (newMessage == null)
         return;

      meshAndMaterialToRender.set(generateMeshAndMaterial(newMessage));

   }

   private Pair<Mesh, Material> generateMeshAndMaterial(PlanarRegionsListMessage newMessage)
   {
      polygonsMeshBuilder.clear();

      double lineWidth = 0.01;
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(newMessage);

      for (int index = 0; index < planarRegionsList.getNumberOfPlanarRegions(); index++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(index);

         int regionId = planarRegion.getRegionId();
         Color regionColor = getRegionColor(regionId);
         planarRegion.getTransformToWorld(transformToWorld);

         polygonsMeshBuilder.addMultiLine(transformToWorld, planarRegion.getConcaveHull(), lineWidth, regionColor, true);


         for (int i = 0; i < planarRegion.getNumberOfConvexPolygons(); i++)
         {
            ConvexPolygon2d convexPolygon2d = planarRegion.getConvexPolygon(i);
            regionColor = Color.hsb(regionColor.getHue(), 0.9, 0.5 + 0.5 * ((double) i / (double) planarRegion.getNumberOfConvexPolygons()));
            polygonsMeshBuilder.addPolygon(transformToWorld, convexPolygon2d, regionColor);
         }
      }

      Material material = polygonsMeshBuilder.generateMaterial();
      Mesh mesh = polygonsMeshBuilder.generateMesh();

      return new Pair<>(mesh, material);
   }

   private Color getRegionColor(int regionId)
   {
      java.awt.Color awtColor = new java.awt.Color(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
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
