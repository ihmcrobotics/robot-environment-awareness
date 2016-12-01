package us.ihmc.octoMap;

import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import javafx.application.Application;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import javafx.util.Pair;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.jOctoMap.tools.JOctoMapRandomTools;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessagerSharedVariables;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.ColoringType;
import us.ihmc.robotEnvironmentAwareness.updaters.REAOcTreeGraphicsBuilder;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionFeatureUpdater;

public class MergingPlanarRegionsVisualizer extends Application
{
   public final NormalOcTree ocTree = new NormalOcTree(0.02);
   private final REAMessager messageManager = new REAMessagerSharedVariables();
   private final REAPlanarRegionFeatureUpdater featureUpdater = new REAPlanarRegionFeatureUpdater(ocTree, messageManager);
   private final REAOcTreeGraphicsBuilder graphicsBuilder = new REAOcTreeGraphicsBuilder(ocTree, featureUpdater, messageManager);

   private final AtomicReference<Pair<Mesh, Material>> occupiedMeshToRender;
   private final AtomicReference<Pair<Mesh, Material>> planarRegionPolygonMeshToRender;

   public MergingPlanarRegionsVisualizer()
   {
      Random random = new Random(23453L);

      occupiedMeshToRender = messageManager.createInput(REAModuleAPI.OcTreeGraphicsOccupiedMesh);
      planarRegionPolygonMeshToRender = messageManager.createInput(REAModuleAPI.OcTreeGraphicsPlanarPolygonMesh);

      Point3d center1 = new Point3d(-5.0, 0.0, 0.0);
      Vector3d normal1 = new Vector3d(0.0, 0.0, 1.0);
      ScanCollection sweep1 = JOctoMapRandomTools.createSingleSweepInPlane(random, 1.0, center1, normal1, 2.0, 2.0, 100000);
      ocTree.update(sweep1);
      Point3d center2 = new Point3d(5.0, 0.0, 0.0);
      Vector3d normal2 = new Vector3d(0.0, 0.0, 1.0);
      ScanCollection sweep2 = JOctoMapRandomTools.createSingleSweepInPlane(random, 1.0, center2, normal2, 2.0, 2.0, 100000);
      ocTree.update(sweep2);

      messageManager.submitMessage(REAModuleAPI.OcTreeEnable, true);
      messageManager.submitMessage(REAModuleAPI.OcTreeGraphicsShowOcTreeNodes, true);
      messageManager.submitMessage(REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces, true);
      messageManager.submitMessage(REAModuleAPI.OcTreeGraphicsColoringMode, ColoringType.REGION);

      messageManager.submitMessage(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerEnable, true);
      

      featureUpdater.update();
      graphicsBuilder.update();
   }
   
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle("OcTree Visualizer");

      View3DFactory view3dFactory = new View3DFactory(600, 400);
      view3dFactory.addCameraController();
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.setRootMouseTransparent(true);

      MeshView occupiedMeshView = new MeshView();
      occupiedMeshView.setMesh(occupiedMeshToRender.get().getKey());
      occupiedMeshView.setMaterial(occupiedMeshToRender.get().getValue());
      view3dFactory.addNodeToView(occupiedMeshView);

      MeshView polygonizedRegionsMeshView = new MeshView();
      polygonizedRegionsMeshView.setMesh(planarRegionPolygonMeshToRender.get().getKey());
      polygonizedRegionsMeshView.setMaterial(planarRegionPolygonMeshToRender.get().getValue());
      view3dFactory.addNodeToView(polygonizedRegionsMeshView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   public static void main(String[] args)
   {
      Application.launch(args);
   }
}
