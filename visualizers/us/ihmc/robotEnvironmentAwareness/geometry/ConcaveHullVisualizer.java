package us.ihmc.robotEnvironmentAwareness.geometry;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.opensphere.geometry.algorithm.ConcaveHull;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.MultiPoint;

import javafx.application.Application;
import javafx.event.Event;
import javafx.scene.Group;
import javafx.scene.PerspectiveCamera;
import javafx.scene.PointLight;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXToolkit.shapes.MultiColorMeshBuilder;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;
import us.ihmc.robotics.time.TimeTools;

public class ConcaveHullVisualizer extends Application
{
   private final List<Point2d> pointCloud = new ArrayList<>();
   private final List<Point2d> concaveHullVertices = new ArrayList<>();
   private final List<Point2d> originalConcaveHullVertices = new ArrayList<>();
   private final MultiColorMeshBuilder meshBuilder = new MultiColorMeshBuilder();
   private ConvexPolygon2d convexPolygon2d;

   public ConcaveHullVisualizer() throws IOException
   {
      InputStreamReader inputStreamReader = new InputStreamReader(getClass().getResourceAsStream("regionPoints"));
      BufferedReader bufferedReader = new BufferedReader(inputStreamReader);

      String line = "";
      String cvsSplitBy = ",";

      while ((line = bufferedReader.readLine()) != null)
      {
         String[] coordsAsString = line.split(cvsSplitBy);
         double x = Double.parseDouble(coordsAsString[0]);
         double y = Double.parseDouble(coordsAsString[1]);
         pointCloud.add(new Point2d(x, y));
      }

      long startTime = System.nanoTime();

      ConcaveHull concaveHull = new ConcaveHull(convertPoint2dToMultipoint(pointCloud), 0.05);
      Geometry concaveHullGeometry = concaveHull.getConcaveHull();

      concaveHullVertices.addAll(getGeometryVertices(concaveHullGeometry));
      originalConcaveHullVertices.addAll(getGeometryVertices(concaveHullGeometry));

      ConcaveHullTools.ensureClockwiseOrdering(concaveHullVertices);

      System.out.println("Size before filtering: " + concaveHullVertices.size());

      int numberOfDuplicateRemoved = ConcaveHullTools.removeSuccessiveDuplicateVertices(concaveHullVertices);
      System.out.println("Removed : " + numberOfDuplicateRemoved + " duplicate vertices");

      long endTime = System.nanoTime();
      System.out.println("ConcaveHull Took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));

      System.out.println("Perimeter: " + ConcaveHullTools.computePerimeter(concaveHullVertices));
      double sd = computeMaxStandardDeviation();
      System.out.println("Standard dev: " + sd);

      double shallowAngleThreshold = Math.toRadians(1.0);
      double peakAngleThreshold = Math.toRadians(120.0);
      double lengthThreshold = 0.10; //sd / 10.0;
      double areaThreshold = 0.001;
      double percentageThreshold = 0.995;
      double depthThreshold = 0.1;

      int nVerticesRemoved = 0;

      startTime = System.nanoTime();
      for (int i = 0; i < 10; i++)
      {
         //                  filter2(sd / 5.0);
//         nVerticesRemoved += ConcaveHullPruningFilteringTools.filterOutGroupsOfShallowVertices(percentageThreshold, concaveHullVertices);
         nVerticesRemoved += ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHullVertices);
         nVerticesRemoved += ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHullVertices);
//         nVerticesRemoved += ConcaveHullPruningFilteringTools.filterOutSmallTriangles(areaThreshold, concaveHullVertices);
//         nVerticesRemoved += ConcaveHullPruningFilteringTools.flattenShallowPockets(depthThreshold, concaveHullVertices);
         
//         filterOutShortEdges(lengthThreshold);
//         filterOutSmallTriangles();
      }
      endTime = System.nanoTime();
      System.out.println("filtering Took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime) + ", number of vertices removed: " + nVerticesRemoved);

      convexPolygon2d = new ConvexPolygon2d(concaveHullVertices);

      startTime = System.nanoTime();
      ConcaveHullDecomposition.decomposeRecursively(concaveHullVertices, depthThreshold, 0, decomposedPolygons);
      endTime = System.nanoTime();
      System.out.println("decomposition Took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime) + ", number of polygons: " + decomposedPolygons.size());

      System.out.println("Size after filtering: " + concaveHullVertices.size());
   }

   private final List<ConvexPolygon2d> decomposedPolygons = new ArrayList<>();

   private double computeMaxStandardDeviation()
   {
      long startTime = System.nanoTime();
      PrincipalComponentAnalysis3D principalComponentAnalysis3D = new PrincipalComponentAnalysis3D();
      principalComponentAnalysis3D.setPointCloud(toPoint3d(pointCloud));
      principalComponentAnalysis3D.compute();
      Vector3d standardDeviation = new Vector3d();
      principalComponentAnalysis3D.getStandardDeviation(standardDeviation);
      long endTime = System.nanoTime();
      System.out.println("Performed PCA in: " + TimeTools.nanoSecondstoSeconds(endTime - startTime));
      return standardDeviation.length();
   }

   private List<Point3d> toPoint3d(List<Point2d> point2ds)
   {
      return toPoint3d(point2ds, 0.0);
   }

   private List<Point3d> toPoint3d(List<Point2d> point2ds, double z)
   {
      List<Point3d> ret = new ArrayList<>();
      for (Point2d point2d : point2ds)
      {
         ret.add(new Point3d(point2d.x, point2d.y, z));
      }
      return ret;
   }

   private void filter2(double threshold)
   {
      int nVerticesRemoved = 0;

      long startTime = System.nanoTime();
      Vector2d cutLine = new Vector2d();
      Vector2d currentToCandidate = new Vector2d();

      for (int i = 0; i < concaveHullVertices.size() - 1; i++)
      {
         Point2d currentVertex = concaveHullVertices.get(i);
         double index = -1;

         for (int j = i + concaveHullVertices.size() / 3; j >= i + 1 + 0 * concaveHullVertices.size() / 6; j--)
         {
            Point2d other = concaveHullVertices.get(j % concaveHullVertices.size());

            if (currentVertex.distance(other) < threshold)
            {
               cutLine.sub(other, currentVertex);
               boolean areAllPointOutside = true;
               for (int k = i + 1; k < j; k++)
               {
                  Point2d candidate = concaveHullVertices.get(k % concaveHullVertices.size());
                  currentToCandidate.sub(candidate, currentVertex);
                  if (cross(cutLine, currentToCandidate) < 0.0)
                  {
                     areAllPointOutside = false;
                     break;
                  }
               }

               if (areAllPointOutside)
               {
                  index = j;
                  break;
               }
            }
         }

         if (index != -1)
         {
            for (int j = i + 1; j < index; j++)
            {
               concaveHullVertices.remove((i + 1) % concaveHullVertices.size());
               nVerticesRemoved++;
            }
         }
         else
            i++;

      }
      long endTime = System.nanoTime();
      System.out.println("filtering bumps took: " + TimeTools.nanoSecondstoSeconds(endTime - startTime) + ", removed " + nVerticesRemoved + " vertices.");
   }

   private MultiPoint convertPoint2dToMultipoint(List<Point2d> points)
   {
      List<Coordinate> coordinates = new ArrayList<>();
      for (Point2d point : points)
      {
         coordinates.add(new Coordinate(point.getX(), point.getY()));
      }
      return new GeometryFactory().createMultiPoint(coordinates.toArray(new Coordinate[0]));
   }

   private List<Point2d> getGeometryVertices(Geometry geometry)
   {
      List<Point2d> vertices = new ArrayList<>();
      for (Coordinate vertex : geometry.getCoordinates())
         vertices.add(new Point2d(vertex.x, vertex.y));
      return vertices;
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
      meshBuilder.addMultiLineMesh(toPoint3d(concaveHullVertices), 0.0025, Color.ALICEBLUE, true);
      //      meshBuilder.addPolyon(getPoint3dsFromPolygon(convexPolygon2d, -0.001), Color.DARKBLUE);

      Random random = new Random(1561L);
      double z = 0.0;
      for (ConvexPolygon2d convexPolygon : decomposedPolygons)
      {
         double hue = 10.0 * 360.0 * random.nextDouble();
         meshBuilder.addPolyon(getPoint3dsFromPolygon(convexPolygon, z += 0.0), Color.hsb(hue, 1.0, 1.0));
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      rootNode.getChildren().add(meshView);

      for (int i = 0; i < concaveHullVertices.size(); i++)
      {
         Sphere sphere = new Sphere(0.008);
         sphere.setMaterial(new PhongMaterial(Color.hsb(240.0 * i / concaveHullVertices.size(), 1.0, 1.0)));
         sphere.setTranslateX(concaveHullVertices.get(i).getX());
         sphere.setTranslateY(concaveHullVertices.get(i).getY());
         rootNode.getChildren().add(sphere);
      }

      for (Point2d vertex : pointCloud)
      {
         Box box = new Box(0.01, 0.01, 0.01);
         boolean partOfOriginalHull = originalConcaveHullVertices.contains(vertex);
         Color color = partOfOriginalHull ? Color.CRIMSON : Color.BLUEVIOLET;
         box.setMaterial(new PhongMaterial(color));
         box.setTranslateX(vertex.getX());
         box.setTranslateY(vertex.getY());
         rootNode.getChildren().add(box);
      }

      for (int i = 0; i < convexPolygon2d.getNumberOfVertices(); i++)
      {
         Sphere sphere = new Sphere(0.008);
         sphere.setMaterial(new PhongMaterial(Color.hsb(240.0 * i / convexPolygon2d.getNumberOfVertices(), 1.0, 1.0)));
         sphere.setTranslateX(convexPolygon2d.getVertex(i).getX());
         sphere.setTranslateY(convexPolygon2d.getVertex(i).getY());
         sphere.setTranslateZ(0.03);
         rootNode.getChildren().add(sphere);
      }
   }

   private List<Point3d> getPoint3dsFromPolygon(ConvexPolygon2d convexPolygon2d, double z)
   {
      List<Point3d> ret = new ArrayList<>();
      for (int i = convexPolygon2d.getNumberOfVertices() - 1; i >= 0; i--)
      {
         Point3d vertex3d = new Point3d();
         Point2d vertex2d = convexPolygon2d.getVertex(i);
         vertex3d.set(vertex2d.getX(), vertex2d.getY(), z);
         ret.add(vertex3d);
      }
      return ret;
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

   private static double cross(Vector2d v1, Vector2d v2)
   {
      return v1.getX() * v2.getY() - v1.getY() * v2.getX();
   }

   public static void main(String[] args)
   {
      Application.launch();
   }
}
