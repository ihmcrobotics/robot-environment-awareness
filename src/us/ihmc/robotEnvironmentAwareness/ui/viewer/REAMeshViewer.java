package us.ihmc.robotEnvironmentAwareness.ui.viewer;

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

public class REAMeshViewer
{
   private final Group root = new Group();

   private final MeshView occupiedLeafsMeshView = new MeshView();
   private final MeshView planarRegionMeshView = new MeshView();
   private Box ocTreeBoundingBoxGraphics = null;

   private final AtomicReference<Pair<Mesh, Material>> occupiedMeshToRender;
   private final AtomicReference<Pair<Mesh, Material>> planarRegionPolygonMeshToRender;
   private final AtomicReference<Box> boundingBoxMeshToRender;

   private final AnimationTimer renderMeshAnimation;

   public REAMeshViewer(REAMessageManager inputManager)
   {
      occupiedMeshToRender = inputManager.createInput(REAModuleAPI.OcTreeGraphicsOccupiedMesh);
      planarRegionPolygonMeshToRender = inputManager.createInput(REAModuleAPI.OcTreeGraphicsPlanarPolygonMesh);
      boundingBoxMeshToRender = inputManager.createInput(REAModuleAPI.OcTreeGraphicsBoundingBoxMesh);

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
               ocTreeBoundingBoxGraphics = boundingBoxMeshToRender.get();
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
