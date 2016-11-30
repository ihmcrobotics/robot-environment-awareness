package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Material;
import javafx.scene.shape.Box;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.BufferOctreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.ScanMeshBuilder;
import us.ihmc.tools.thread.ThreadTools;

public class REAMeshViewer
{
   private final int SCAN_MESH_BUILDER_RUN_PERIOD_MILLISECONDS = 10;
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

   private ScheduledExecutorService executorService = Executors.newScheduledThreadPool(3, ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final AnimationTimer renderMeshAnimation;

   private final ScanMeshBuilder scanMeshBuilder;
   private final BufferOctreeMeshBuilder bufferOctreeMeshBuilder;
   private final OcTreeMeshBuilder ocTreeMeshBuilder;

   public REAMeshViewer(REAMessager reaMessager)
   {
      occupiedMeshToRender = new AtomicReference<>(null);
      planarRegionPolygonMeshToRender = new AtomicReference<>(null);
      bufferMeshToRender = new AtomicReference<>(null);
      scanInputMeshToRender = new AtomicReference<>(null);

      boundingBoxMeshToRender = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsBoundingBoxMesh);

      // TEST Communication over network
      scanMeshBuilder = new ScanMeshBuilder(reaMessager, scanMeshBuilderListener);
      bufferOctreeMeshBuilder = new BufferOctreeMeshBuilder(reaMessager, bufferOctreeMeshBuilderListener);
      ocTreeMeshBuilder = new OcTreeMeshBuilder(reaMessager, octreeMeshBuilderListener);

      root.getChildren().addAll(occupiedLeafsMeshView, bufferLeafsMeshView, scanInputMeshView, planarRegionMeshView);
      root.setMouseTransparent(true);

      renderMeshAnimation = new AnimationTimer()
      {
         @Override public void handle(long now)
         {
            if (Thread.interrupted())
               return;

            if (occupiedMeshToRender.get() != null)
            {
               System.out.println("Updating occupied mesh view");
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
      executorService.scheduleAtFixedRate(scanMeshBuilder, 0, SCAN_MESH_BUILDER_RUN_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      executorService.scheduleAtFixedRate(bufferOctreeMeshBuilder, 0, SCAN_MESH_BUILDER_RUN_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      executorService.scheduleAtFixedRate(ocTreeMeshBuilder, 0, SCAN_MESH_BUILDER_RUN_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
   }

   public void stop()
   {
      renderMeshAnimation.stop();

      if (executorService != null)
      {
         executorService.shutdown();
         executorService = null;
      }
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

   private ScanMeshBuilder.ScanMeshBuilderListener scanMeshBuilderListener = new ScanMeshBuilder.ScanMeshBuilderListener()
   {
      @Override public void scanMeshAndMaterialChanged(Pair<Mesh, Material> meshMaterial)
      {
         scanInputMeshToRender.set(meshMaterial);
      }
   };

   private BufferOctreeMeshBuilder.BufferOctreeMeshBuilderListener bufferOctreeMeshBuilderListener = new BufferOctreeMeshBuilder.BufferOctreeMeshBuilderListener()
   {
      @Override public void meshAndMaterialChanged(Pair<Mesh, Material> meshMaterial)
      {
         bufferMeshToRender.set(meshMaterial);
      }
   };

   private OcTreeMeshBuilder.OctreeMeshBuilderListener octreeMeshBuilderListener = new OcTreeMeshBuilder.OctreeMeshBuilderListener()
   {
      @Override public void occupiedMeshAndMaterialChanged(Pair<Mesh, Material> meshAndMaterial)
      {
         occupiedMeshToRender.set(meshAndMaterial);
      }

      @Override public void planarRegionMeshAndMaterialChanged(Pair<Mesh, Material> meshAndMaterial)
      {
         planarRegionPolygonMeshToRender.set(meshAndMaterial);
      }

   };

}
