package us.ihmc.octoMap;

import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import javafx.application.Application;
import javafx.event.Event;
import javafx.scene.Group;
import javafx.scene.PerspectiveCamera;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import javafx.util.Pair;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.octoMap.ocTree.NormalOcTree;
import us.ihmc.octoMap.pointCloud.SweepCollection;
import us.ihmc.octoMap.tools.OctoMapRandomTools;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessageManager;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.updaters.REAOcTreeGraphicsBuilder;
import us.ihmc.robotEnvironmentAwareness.updaters.REAOcTreeGraphicsBuilder.ColoringType;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionFeatureUpdater;

public class MergingPlanarRegionsVisualizer extends Application
{
   public final NormalOcTree ocTree = new NormalOcTree(0.02);
   private final REAMessageManager inputManager = new REAMessageManager();
   private final REAMessageManager outputManager = new REAMessageManager();
   private final REAPlanarRegionFeatureUpdater featureUpdater = new REAPlanarRegionFeatureUpdater(ocTree, inputManager, outputManager);
   private final REAOcTreeGraphicsBuilder graphicsBuilder = new REAOcTreeGraphicsBuilder(ocTree, featureUpdater, inputManager, outputManager);

   private final AtomicReference<Pair<Mesh, Material>> occupiedMeshToRender;
   private final AtomicReference<Pair<Mesh, Material>> planarRegionPolygonMeshToRender;

   public MergingPlanarRegionsVisualizer()
   {
      Random random = new Random(23453L);

      occupiedMeshToRender = outputManager.createInput(REAModuleAPI.OcTreeGraphicsOccupiedMesh);
      planarRegionPolygonMeshToRender = outputManager.createInput(REAModuleAPI.OcTreeGraphicsPlanarPolygonMesh);

      Point3d center1 = new Point3d(-5.0, 0.0, 0.0);
      Vector3d normal1 = new Vector3d(0.0, 0.0, 1.0);
      SweepCollection sweep1 = OctoMapRandomTools.createSingleSweepInPlane(random, 1.0, center1, normal1, 2.0, 2.0, 100000);
      ocTree.update(sweep1);
      Point3d center2 = new Point3d(5.0, 0.0, 0.0);
      Vector3d normal2 = new Vector3d(0.0, 0.0, 1.0);
      SweepCollection sweep2 = OctoMapRandomTools.createSingleSweepInPlane(random, 1.0, center2, normal2, 2.0, 2.0, 100000);
      ocTree.update(sweep2);

      inputManager.submitMessage(new REAMessage(REAModuleAPI.OcTreeEnable, true));
      inputManager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsShowOcTreeNodes, true));
      inputManager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces, true));
      inputManager.submitMessage(new REAMessage(REAModuleAPI.OcTreeGraphicsColoringMode, ColoringType.REGION));

      inputManager.submitMessage(new REAMessage(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerEnable, true));
      

      featureUpdater.update();
      graphicsBuilder.update();
   }
   
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle("OcTree Visualizer");

      Group rootNode = new Group();
      Scene scene = new Scene(rootNode, 600, 400, true);
      scene.setFill(Color.GRAY);
      rootNode.setMouseTransparent(true);
      setupCamera(rootNode, scene);
      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(0.3);
      rootNode.getChildren().add(worldCoordinateSystem);

      primaryStage.setScene(scene);
      primaryStage.show();

      MeshView occupiedMeshView = new MeshView();
      occupiedMeshView.setMesh(occupiedMeshToRender.get().getKey());
      occupiedMeshView.setMaterial(occupiedMeshToRender.get().getValue());
      rootNode.getChildren().add(occupiedMeshView);

      MeshView polygonizedRegionsMeshView = new MeshView();
      polygonizedRegionsMeshView.setMesh(planarRegionPolygonMeshToRender.get().getKey());
      polygonizedRegionsMeshView.setMaterial(planarRegionPolygonMeshToRender.get().getValue());
      rootNode.getChildren().add(polygonizedRegionsMeshView);
   }

   private void setupCamera(Group root, Scene scene)
   {
      PerspectiveCamera camera = new PerspectiveCamera(true);
      camera.setNearClip(0.05);
      camera.setFarClip(50.0);
      scene.setCamera(camera);

      Vector3d up = new Vector3d(0.0, 0.0, 1.0);
      FocusBasedCameraMouseEventHandler cameraController = new FocusBasedCameraMouseEventHandler(scene.widthProperty(), scene.heightProperty(), camera, up);
      scene.addEventHandler(Event.ANY, cameraController);
      root.getChildren().add(cameraController.getFocusPointViz());
   }

   public static void main(String[] args)
   {
      Application.launch(args);
   }
}
