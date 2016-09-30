package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Material;
import javafx.scene.shape.Box;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessageManager;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleAPI;

public class REAMeshViewController
{
   private final Group root = new Group();

   private final MeshView occupiedLeafsMeshView = new MeshView();
   private final MeshView planarRegionMeshView = new MeshView();
   private Box ocTreeBoundingBoxGraphics = null;

   private final AtomicReference<Pair<Mesh, Material>> occupiedMeshToRender;
   private final AtomicReference<Pair<Mesh, Material>> planarRegionPolygonMeshToRender;
   private final AtomicReference<Box> boundingBoxMeshToRender;

   private final AnimationTimer renderMeshAnimation;

   @SuppressWarnings("unchecked")
   public REAMeshViewController(REAMessageManager inputManager)
   {
      Pair<Mesh, Material> pair = new Pair<Mesh, Material>(null, null);
      occupiedMeshToRender = (AtomicReference<Pair<Mesh, Material>>) inputManager.createInput(REAModuleAPI.OcTreeGraphicsOccupiedMesh, pair.getClass());
      planarRegionPolygonMeshToRender = (AtomicReference<Pair<Mesh, Material>>) inputManager.createInput(REAModuleAPI.OcTreeGraphicsPlanarPolygonMesh, pair.getClass());
      boundingBoxMeshToRender = inputManager.createInput(REAModuleAPI.OcTreeGraphicsBoundingBoxMesh, Box.class);

      root.getChildren().addAll(occupiedLeafsMeshView, planarRegionMeshView);
      root.setMouseTransparent(true);

      renderMeshAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            if (Thread.interrupted())
               return;

            if (occupiedMeshToRender.get() != null)
            {
               updateMeshView(occupiedLeafsMeshView, occupiedMeshToRender.getAndSet(null));
            }

            if (planarRegionPolygonMeshToRender.get() != null)
            {
               updateMeshView(planarRegionMeshView, planarRegionPolygonMeshToRender.getAndSet(null));
            }

            if (ocTreeBoundingBoxGraphics != null)
            {
               root.getChildren().remove(ocTreeBoundingBoxGraphics);
               ocTreeBoundingBoxGraphics = null;
            }

            if (boundingBoxMeshToRender.get() != null)
            {
               ocTreeBoundingBoxGraphics = boundingBoxMeshToRender.getAndSet(null);
               root.getChildren().add(ocTreeBoundingBoxGraphics);
            }
         }
      };
   }

   public void start()
   {
      renderMeshAnimation.start();
   }

   public void stop()
   {
      renderMeshAnimation.stop();
   }

   private void updateMeshView(MeshView meshViewToUpdate, Pair<Mesh, Material> meshMaterial)
   {
      meshViewToUpdate.setMesh(meshMaterial.getKey());
      meshViewToUpdate.setMaterial(meshMaterial.getValue());
   }

   public Node getRoot()
   {
      return root;
   }
}
