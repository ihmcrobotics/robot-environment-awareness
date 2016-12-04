package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.BoundingBoxMeshView;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.BufferOctreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.PlanarRegionsMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.ScanMeshBuilder;

public class REAMeshViewer
{
   private static final int LOW_RATE_UPDATE_PERIOD = 2000;
   private static final int HIGH_RATE_UPDATE_PERIOD = 10;

   private final Group root = new Group();

   private final MeshView occupiedLeafsMeshView = new MeshView();
   private final MeshView bufferLeafsMeshView = new MeshView();
   private final MeshView scanInputMeshView = new MeshView();
   private final MeshView planarRegionMeshView = new MeshView();

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(4, getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   private final AnimationTimer renderMeshAnimation;

   private final ScanMeshBuilder scanMeshBuilder;
   private final BufferOctreeMeshBuilder bufferOctreeMeshBuilder;
   private final OcTreeMeshBuilder ocTreeMeshBuilder;
   private final PlanarRegionsMeshBuilder planarRegionsMeshBuilder;
   private final BoundingBoxMeshView boundingBoxMeshView;

   public REAMeshViewer(REAUIMessager uiMessager)
   {
      // TEST Communication over network
      scanMeshBuilder = new ScanMeshBuilder(uiMessager);
      bufferOctreeMeshBuilder = new BufferOctreeMeshBuilder(uiMessager);
      ocTreeMeshBuilder = new OcTreeMeshBuilder(uiMessager);
      boundingBoxMeshView = new BoundingBoxMeshView(uiMessager);
      planarRegionsMeshBuilder = new PlanarRegionsMeshBuilder(uiMessager);

      root.getChildren().addAll(occupiedLeafsMeshView, bufferLeafsMeshView, scanInputMeshView, planarRegionMeshView, boundingBoxMeshView);
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
         }
      };
   }

   public void start()
   {
      renderMeshAnimation.start();
      executorService.scheduleAtFixedRate(scanMeshBuilder, 0, HIGH_RATE_UPDATE_PERIOD, TimeUnit.MILLISECONDS);
      executorService.scheduleAtFixedRate(bufferOctreeMeshBuilder, 0, HIGH_RATE_UPDATE_PERIOD, TimeUnit.MILLISECONDS);
      executorService.scheduleAtFixedRate(ocTreeMeshBuilder, 0, LOW_RATE_UPDATE_PERIOD, TimeUnit.MILLISECONDS);
      executorService.scheduleAtFixedRate(boundingBoxMeshView, 0, LOW_RATE_UPDATE_PERIOD, TimeUnit.MILLISECONDS);
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
