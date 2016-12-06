package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3f;

import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.util.Pair;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.packets.LineSegment3dMessage;

public class PlanarRegionsIntersectionsMeshBuilder implements Runnable
{
   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> clear;

   private final AtomicReference<LineSegment3dMessage[]> intersectionsMessage;

   private final JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
   private final Material material = new PhongMaterial(Color.BLACK);
   private final REAUIMessager uiMessager;

   private final AtomicReference<Pair<Mesh, Material>> meshAndMaterialToRender = new AtomicReference<>(null);

   public PlanarRegionsIntersectionsMeshBuilder(REAUIMessager uiMessager)
   {
      this.uiMessager = uiMessager;
      enable = uiMessager.createInput(REAModuleAPI.OcTreeEnable, false);
      clear = uiMessager.createInput(REAModuleAPI.OcTreeClear, false);

      intersectionsMessage = uiMessager.createInput(REAModuleAPI.PlanarRegionsIntersectionState);
   }

   @Override
   public void run()
   {
      LineSegment3dMessage[] newMessage = intersectionsMessage.getAndSet(null);

      if (clear.getAndSet(false))
      {
         meshAndMaterialToRender.set(new Pair<>(null, null));
         return;
      }

      if (!enable.get())
         return;

      uiMessager.submitStateRequestToModule(REAModuleAPI.RequestPlanarRegionsIntersections);

      if (newMessage == null)
         return;

      for (LineSegment3dMessage intersection : newMessage)
      {
         Point3f start = intersection.getStart();
         Point3f end = intersection.getEnd();
         float lineWidth = 0.025f;
         meshBuilder.addLine(start, end, lineWidth);
      }

      meshAndMaterialToRender.set(new Pair<Mesh, Material>(meshBuilder.generateMesh(), material));
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