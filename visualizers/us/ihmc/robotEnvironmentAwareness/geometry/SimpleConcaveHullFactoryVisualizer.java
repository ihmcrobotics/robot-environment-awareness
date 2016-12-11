package us.ihmc.robotEnvironmentAwareness.geometry;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

public class SimpleConcaveHullFactoryVisualizer extends Application
{
   private static final boolean VISUALIZE_PC3D = false;

   private final List<Point3d> pointCloud3d = new ArrayList<>();
   private final List<Point2d> pointCloud = new ArrayList<>();
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder();

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

      View3DFactory view3dFactory = new View3DFactory(600, 400);
      view3dFactory.addCameraController();
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addPointLight(0.0, 0.0, 2.0);

      meshBuilder.clear();

      List<Point3d> concaveHullVertices = SimpleConcaveHullFactory.createConcaveHull(pointCloud, 0.10).getOrderedConcaveHullVertices(0.0);
      meshBuilder.addMultiLine(concaveHullVertices, 0.005, Color.TURQUOISE, true);

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      meshView.setMouseTransparent(true);
      view3dFactory.addNodeToView(meshView);

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
         view3dFactory.addNodeToView(box);
      }

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   public static void main(String[] args)
   {
      Application.launch();
   }
}
