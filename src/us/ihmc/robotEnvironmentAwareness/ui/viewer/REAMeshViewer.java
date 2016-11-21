package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import java.io.IOException;
import java.util.ArrayList;
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
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationKryoNetClassList;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.BufferOctreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.ScanMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessager;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessagerOverNetwork;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleAPI;
import us.ihmc.tools.thread.ThreadTools;

import javax.vecmath.Point3f;

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


   public REAMeshViewer(REAMessager reaMessager, REAMessager reaMessagerOverNetwork)
   {
      occupiedMeshToRender = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsOccupiedMesh);
      bufferMeshToRender = new AtomicReference<>(null);

      scanInputMeshToRender = new AtomicReference<>(null);

      planarRegionPolygonMeshToRender = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsPlanarPolygonMesh);
      boundingBoxMeshToRender = reaMessager.createInput(REAModuleAPI.OcTreeGraphicsBoundingBoxMesh);

      // TEST Communication over network
      scanMeshBuilder = new ScanMeshBuilder(reaMessagerOverNetwork, scanMeshBuilderListener);
      bufferOctreeMeshBuilder = new BufferOctreeMeshBuilder(reaMessagerOverNetwork, bufferOctreeMeshBuilderListener);

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
      executorService.scheduleAtFixedRate(scanMeshBuilder, 0, SCAN_MESH_BUILDER_RUN_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      executorService.scheduleAtFixedRate(bufferOctreeMeshBuilder, 0, SCAN_MESH_BUILDER_RUN_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
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

}
