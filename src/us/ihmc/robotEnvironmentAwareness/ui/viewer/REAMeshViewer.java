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
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessager;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleAPI;

public class REAMeshViewer
{
   private final Group root = new Group();

   private final MeshView occupiedLeafsMeshView = new MeshView();
   private final MeshView bufferLeafsMeshView = new MeshView();
   private final MeshView scanInputMeshView = new MeshView();
   private final MeshView planarRegionMeshView = new MeshView();
   private Box ocTreeBoundingBoxGraphics = null;

   private final AtomicReference<Pair<Mesh, Material>> occupiedMeshToRender;
   private final AtomicReference<Pair<Mesh, Material>> bufferMeshToRender;
   private final AtomicReference<Pair<Mesh, Material>> scanInputMeshToRender;
   private final AtomicReference<Pair<Mesh, Material>> planarRegionPolygonMeshToRender;
   private final AtomicReference<Box> boundingBoxMeshToRender;

   private final AnimationTimer renderMeshAnimation;

   public REAMeshViewer(REAMessager reaMessager)
   {
      occupiedMeshToRender = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsOccupiedMesh);
      bufferMeshToRender = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsBufferMesh);
      scanInputMeshToRender = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsInputScanMesh);
      planarRegionPolygonMeshToRender = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsPlanarPolygonMesh);
      boundingBoxMeshToRender = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsBoundingBoxMesh);

      root.getChildren().addAll(occupiedLeafsMeshView, bufferLeafsMeshView, scanInputMeshView, planarRegionMeshView);
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

            if (bufferMeshToRender.get() != null)
            {
               updateMeshView(bufferLeafsMeshView, bufferMeshToRender.getAndSet(null));
            }

            if (scanInputMeshToRender.get() != null)
            {
               updateMeshView(scanInputMeshView, scanInputMeshToRender.getAndSet(null));
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
