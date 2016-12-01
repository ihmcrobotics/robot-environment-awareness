package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Material;
import javafx.scene.shape.Box;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.BoundingBoxMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.BufferOctreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.PlanarRegionsMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.ScanMeshBuilder;
import us.ihmc.tools.thread.ThreadTools;

public class REAMeshViewer
{
   private static final int LOW_RATE_UPDATE_PERIOD = 2000;
   private static final int HIGH_RATE_UPDATE_PERIOD = 10;

   private final Group root = new Group();

   private final MeshView occupiedLeafsMeshView = new MeshView();
   private final MeshView bufferLeafsMeshView = new MeshView();
   private final MeshView scanInputMeshView = new MeshView();
   private final MeshView planarRegionMeshView = new MeshView();
   private Box ocTreeBoundingBoxGraphics = null;

   private ScheduledExecutorService executorService = Executors.newScheduledThreadPool(4, ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final AnimationTimer renderMeshAnimation;

   private final ScanMeshBuilder scanMeshBuilder;
   private final BufferOctreeMeshBuilder bufferOctreeMeshBuilder;
   private final OcTreeMeshBuilder ocTreeMeshBuilder;
   private final BoundingBoxMeshBuilder boundingBoxMeshBuilder;
   private final PlanarRegionsMeshBuilder planarRegionsMeshBuilder;

   public REAMeshViewer(REAMessager reaMessager)
   {
      // TEST Communication over network
      scanMeshBuilder = new ScanMeshBuilder(reaMessager);
      bufferOctreeMeshBuilder = new BufferOctreeMeshBuilder(reaMessager);
      ocTreeMeshBuilder = new OcTreeMeshBuilder(reaMessager);
      boundingBoxMeshBuilder = new BoundingBoxMeshBuilder(reaMessager);
      planarRegionsMeshBuilder = new PlanarRegionsMeshBuilder(reaMessager);

      root.getChildren().addAll(occupiedLeafsMeshView, bufferLeafsMeshView, scanInputMeshView, planarRegionMeshView);
      root.setMouseTransparent(true);

      renderMeshAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            if (ocTreeMeshBuilder.hasNewMeshAndMaterial())
            {
               updateMeshView(occupiedLeafsMeshView, ocTreeMeshBuilder.pollMeshAndMaterial());
            }

            if (bufferOctreeMeshBuilder.hasNewMeshAndMaterial())
            {
               updateMeshView(bufferLeafsMeshView, bufferOctreeMeshBuilder.pollMeshAndMaterial());
            }

            if (scanMeshBuilder.hasNewMeshAndMaterial())
            {
               updateMeshView(scanInputMeshView, scanMeshBuilder.pollMeshAndMaterial());
            }

            if (planarRegionsMeshBuilder.hasNewMeshAndMaterial())
            {
               updateMeshView(planarRegionMeshView, planarRegionsMeshBuilder.pollMeshAndMaterial());
            }

            if (ocTreeBoundingBoxGraphics != null)
            {
               root.getChildren().remove(ocTreeBoundingBoxGraphics);
               ocTreeBoundingBoxGraphics = null;
            }

            if (boundingBoxMeshBuilder.hasBoundingBoxToRender())
            {
               ocTreeBoundingBoxGraphics = boundingBoxMeshBuilder.getBoundingBox();
               root.getChildren().add(ocTreeBoundingBoxGraphics);
            }
         }
      };
   }

   public void start()
   {
      renderMeshAnimation.start();
      executorService.scheduleAtFixedRate(scanMeshBuilder, 0, HIGH_RATE_UPDATE_PERIOD, TimeUnit.MILLISECONDS);
      executorService.scheduleAtFixedRate(bufferOctreeMeshBuilder, 0, HIGH_RATE_UPDATE_PERIOD, TimeUnit.MILLISECONDS);
      executorService.scheduleAtFixedRate(ocTreeMeshBuilder, 0, LOW_RATE_UPDATE_PERIOD, TimeUnit.MILLISECONDS);
      executorService.scheduleAtFixedRate(boundingBoxMeshBuilder, 0, LOW_RATE_UPDATE_PERIOD, TimeUnit.MILLISECONDS);
      executorService.scheduleAtFixedRate(planarRegionsMeshBuilder, 0, LOW_RATE_UPDATE_PERIOD, TimeUnit.MILLISECONDS);
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
}
