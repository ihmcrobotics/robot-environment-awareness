package us.ihmc.robotEnvironmentAwareness.geometry;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import javafx.application.Application;
import javafx.event.Event;
import javafx.scene.Group;
import javafx.scene.PerspectiveCamera;
import javafx.scene.PointLight;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXToolkit.shapes.MultiColorMeshBuilder;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

public class SimpleConcaveHullFactoryVisualizer extends Application
{
   private static final boolean VISUALIZE_PC3D = false;

   private final List<Point3d> pointCloud3d = new ArrayList<>();
   private final List<Point2d> pointCloud = new ArrayList<>();
   private final MultiColorMeshBuilder meshBuilder = new MultiColorMeshBuilder();

   public SimpleConcaveHullFactoryVisualizer() throws IOException
   {
      createConcaveHullFromRegion("regionPointsCube1");
//      createConcaveHullFromRegion("regionPoints");
   }

   public void createConcaveHullFromRegion(String fileName) throws IOException
   {
      InputStreamReader inputStreamReader = new InputStreamReader(getClass().getResourceAsStream(fileName));
      BufferedReader bufferedReader = new BufferedReader(inputStreamReader);

      String line = "";
      String cvsSplitBy = ",";

      
      while ((line = bufferedReader.readLine()) != null)
      {
         String[] coordsAsString = line.split(cvsSplitBy);
         double x = Double.parseDouble(coordsAsString[0]);
         double y = Double.parseDouble(coordsAsString[1]);
         double z = Double.parseDouble(coordsAsString[2]);
         Point3d e = new Point3d(x, y, z);
         e.scale(1.5);
         pointCloud3d.add(e);
      }

      Point3d average = GeometryTools.averagePoint3ds(pointCloud3d);

      for (Point3d point3d : pointCloud3d)
      {
         point3d.sub(average);
      }

      PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
      pca.setPointCloud(pointCloud3d);
      pca.compute();
      Matrix3d rotationMatrix = new Matrix3d();
      pca.getPrincipalFrameRotationMatrix(rotationMatrix);
      rotationMatrix.transpose();

      for (Point3d point3d : pointCloud3d)
      {
         rotationMatrix.transform(point3d);
         pointCloud.add(new Point2d(point3d.getX(), point3d.getY()));
      }
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

      PointLight light = new PointLight(Color.WHITE);
      light.setTranslateZ(2.0);
      rootNode.getChildren().add(light);

      meshBuilder.clear();

      SimpleConcaveHullFactory concaveHullFactory = new SimpleConcaveHullFactory();
      concaveHullFactory.setEdgeLengthThreshold(0.10);
      concaveHullFactory.setPointCloud(pointCloud);
      concaveHullFactory.computeConcaveHull();

      meshBuilder.addMultiLineMesh(concaveHullFactory.concaveHullAsListOfPoint3d(), 0.005, Color.TURQUOISE, true);

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      rootNode.getChildren().add(meshView);

      for (int i = 0; i < pointCloud.size(); i++)
      {
         Point2d vertex = pointCloud.get(i);
         Box box = new Box(0.01, 0.01, 0.01);
         Color color = Color.BLUEVIOLET;
         box.setMaterial(new PhongMaterial(color));
         if (VISUALIZE_PC3D)
         {
            Point3d vertex3d = pointCloud3d.get(i);
            box.setTranslateX(vertex3d.getX());
            box.setTranslateY(vertex3d.getY());
            box.setTranslateZ(vertex3d.getZ());
         }
         else
         {
            box.setTranslateX(vertex.getX());
            box.setTranslateY(vertex.getY());
         }
         rootNode.getChildren().add(box);
      }
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
      Application.launch();
   }
}
