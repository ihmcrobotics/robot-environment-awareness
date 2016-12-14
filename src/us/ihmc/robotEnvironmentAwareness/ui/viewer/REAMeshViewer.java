package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.BoundingBoxMeshView;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.BufferOctreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.PlanarRegionsIntersectionsMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.PlanarRegionsMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.LidarScanViewer;

public class REAMeshViewer
{
   private static final int SLOW_PACE_UPDATE_PERIOD = 2000;
   private static final int MEDIUM_PACE_UPDATE_PERIOD = 100;
   private static final int HIGH_PACE_UPDATE_PERIOD = 10;

   private final Group root = new Group();

   private final List<ScheduledFuture<?>> meshBuilderScheduledFutures = new ArrayList<>();
   private final MeshView ocTreeMeshView = new MeshView();
   private final MeshView bufferOcTreeMeshView = new MeshView();
   private final MeshView planarRegionMeshView = new MeshView();
   private final MeshView intersectionsMeshView = new MeshView();

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(2, getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   private final AnimationTimer renderMeshAnimation;

   private final LidarScanViewer lidarScanViewer;
   private final BufferOctreeMeshBuilder bufferOctreeMeshBuilder;
   private final OcTreeMeshBuilder ocTreeMeshBuilder;
   private final PlanarRegionsMeshBuilder planarRegionsMeshBuilder;
   private final PlanarRegionsIntersectionsMeshBuilder intersectionsMeshBuilder;
   private final BoundingBoxMeshView boundingBoxMeshView;

   public REAMeshViewer(REAUIMessager uiMessager)
   {
      // TEST Communication over network
      lidarScanViewer = new LidarScanViewer(uiMessager);
      bufferOctreeMeshBuilder = new BufferOctreeMeshBuilder(uiMessager);
      ocTreeMeshBuilder = new OcTreeMeshBuilder(uiMessager);
      planarRegionsMeshBuilder = new PlanarRegionsMeshBuilder(uiMessager);
      intersectionsMeshBuilder = new PlanarRegionsIntersectionsMeshBuilder(uiMessager);
      boundingBoxMeshView = new BoundingBoxMeshView(uiMessager);

      root.getChildren().addAll(lidarScanViewer.getRoot(), bufferOcTreeMeshView, ocTreeMeshView, planarRegionMeshView, intersectionsMeshView, boundingBoxMeshView);
      root.setMouseTransparent(true);

      renderMeshAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            lidarScanViewer.render();

            if (bufferOctreeMeshBuilder.hasNewMeshAndMaterial())
               updateMeshView(bufferOcTreeMeshView, bufferOctreeMeshBuilder.pollMeshAndMaterial());

            if (ocTreeMeshBuilder.hasNewMeshAndMaterial())
               updateMeshView(ocTreeMeshView, ocTreeMeshBuilder.pollMeshAndMaterial());

            if (planarRegionsMeshBuilder.hasNewMeshAndMaterial())
               updateMeshView(planarRegionMeshView, planarRegionsMeshBuilder.pollMeshAndMaterial());

            if (intersectionsMeshBuilder.hasNewMeshAndMaterial())
               updateMeshView(intersectionsMeshView, intersectionsMeshBuilder.pollMeshAndMaterial());
         }
      };

      uiMessager.registerModuleConnectionStateListener(new NetStateListener()
      {
         @Override
         public void disconnected()
         {
            sleep();
         }
         
         @Override
         public void connected()
         {
            start();
         }
      });
   }

   public void start()
   {
      if (!meshBuilderScheduledFutures.isEmpty())
         return;
      renderMeshAnimation.start();
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(lidarScanViewer, 0, HIGH_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(bufferOctreeMeshBuilder, 0, HIGH_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(ocTreeMeshBuilder, 0, SLOW_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(planarRegionsMeshBuilder, 0, MEDIUM_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(intersectionsMeshBuilder, 0, MEDIUM_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(boundingBoxMeshView, 0, MEDIUM_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
   }

   public void sleep()
   {
      if (meshBuilderScheduledFutures.isEmpty())
         return;
      renderMeshAnimation.stop();
      for (ScheduledFuture<?> scheduledFuture : meshBuilderScheduledFutures)
         scheduledFuture.cancel(true);
      meshBuilderScheduledFutures.clear();
   }

   public void stop()
   {
      sleep();

      if (executorService != null)
      {
         executorService.shutdownNow();
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
