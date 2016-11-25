package us.ihmc.octoMap;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import javafx.stage.Stage;
import us.ihmc.jOctoMap.iterators.OcTreeIterable;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.robotEnvironmentAwareness.geometry.IntersectionPlaneBoxCalculator;
import us.ihmc.robotEnvironmentAwareness.tools.View3DFactory;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.time.TimeTools;

public class NormalOcTreeVisualizer extends Application
{
   public final NormalOcTree ocTree = new NormalOcTree(0.025);
   private static final boolean SHOW_FREE_CELLS = false;
   private static final Color FREE_COLOR = new Color(Color.YELLOW.getRed(), Color.YELLOW.getGreen(), Color.YELLOW.getBlue(), 0.0);
   private static final boolean SHOW_POINT_CLOUD = true;
   private static final boolean SHOW_HIT_LOCATIONS = true;

   public NormalOcTreeVisualizer()
   {

      Point3d lidarPosition = new Point3d(0.0, 0.0, 2.0);
      //      callUpdateNode();
      //      callInsertPointCloud();
      createPlane(lidarPosition, 12.0, 0.0, -0.05);
//      createBowl(0.5, new Point3d());
      System.out.println("Number of leafs: " + ocTree.getNumberOfLeafNodes());
      System.out.println("Initialized octree");
      System.out.println("Computing normals");
      long startTime = System.nanoTime();
      ocTree.update(new ScanCollection(pointcloud, lidarPosition));
      ocTree.update(new ScanCollection(pointcloud, lidarPosition));
      ocTree.update(new ScanCollection(pointcloud, lidarPosition));
      ocTree.update(new ScanCollection(pointcloud, lidarPosition));
      ocTree.update(new ScanCollection(pointcloud, lidarPosition));
      ocTree.update(new ScanCollection(pointcloud, lidarPosition));
      long endTime = System.nanoTime();
      System.out.println("Done computing normals: time it took = " + TimeTools.nanoSecondstoSeconds(endTime - startTime));
   }
   
   PointCloud pointcloud = new PointCloud();

   private void callInsertPointCloud()
   {
      Point3d origin = new Point3d(0.01, 0.01, 0.02);
      Point3d pointOnSurface = new Point3d(4.01, 0.01, 0.01);

      for (int i = 0; i < 100; i++)
      {
         for (int j = 0; j < 100; j++)
         {
            Point3d rotated = new Point3d(pointOnSurface);
            Matrix3d rotation = new Matrix3d();
            RotationTools.convertYawPitchRollToMatrix(Math.toRadians(i * 0.5), Math.toRadians(j * 0.5), 0.0, rotation);
            rotation.transform(rotated);
            pointcloud.add(rotated);
         }
      }

      ocTree.update(new ScanCollection(pointcloud, origin));
   }

   public void createPlane(Point3d lidarPosition, double pitch, double roll, double z)
   {
      pointcloud.clear();

      double planeSize = 1.00;

      for (double x = -0.5 * planeSize; x < 0.5 * planeSize; x += 0.7 * ocTree.getResolution())
      {
         for (double y = -0.5 * planeSize; y < 0.5 * planeSize; y += 0.7 * ocTree.getResolution())
         {
            Point3d point = new Point3d(x, y, 0.0);
            Matrix3d rotation = new Matrix3d();
            RotationTools.convertYawPitchRollToMatrix(0.0, Math.toRadians(pitch), Math.toRadians(roll), rotation);
            rotation.transform(point);
            point.setZ(point.getZ() + z);

            pointcloud.add(point);
         }
      }
      ocTree.update(new ScanCollection(pointcloud, lidarPosition));
   }

   public void createBowl(double radius, Point3d center)
   {
      Point3d origin = new Point3d(0.0, 0.0, center.getZ() + 0.0);

      double res = 0.05;
      for (double yaw = 0.0; yaw < 2.0 * Math.PI; yaw += res)
      {
         for (double pitch = 0.0; pitch < 0.5 * Math.PI; pitch += res)
         {
            double x = Math.cos(pitch) * Math.cos(yaw) * radius + center.getX();
            double y = Math.cos(pitch) * Math.sin(yaw) * radius + center.getY();
            double z = - Math.sin(pitch) * radius + center.getZ();
            pointcloud.add(x, y, z);
         }
      }

      ocTree.update(new ScanCollection(pointcloud, origin));
   }

   private final RecyclingArrayList<Point3d> plane = new RecyclingArrayList<>(GenericTypeBuilder.createBuilderWithEmptyConstructor(Point3d.class));
   private final IntersectionPlaneBoxCalculator intersectionPlaneBoxCalculator = new IntersectionPlaneBoxCalculator();

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle("OcTree Visualizer");

      View3DFactory view3dFactory = new View3DFactory(600, 400);
      view3dFactory.addCameraController();
      view3dFactory.addWorldCoordinateSystem(0.3);

      TextureColorPalette1D palette = new TextureColorPalette1D();
      palette.setHueBased(0.9, 0.8);
      JavaFXMultiColorMeshBuilder occupiedMeshBuilder = new JavaFXMultiColorMeshBuilder(palette);
      JavaFXMeshBuilder freeMeshBuilder = new JavaFXMeshBuilder();

      OcTreeIterable<NormalOcTreeNode> leafIterable = OcTreeIteratorFactory.createLeafIteratable(ocTree.getRoot(), 14);
      for (NormalOcTreeNode node : leafIterable)
      {
         double boxSize = node.getSize();
         Point3d nodeCenter = new Point3d();
         node.getCoordinate(nodeCenter);

         if (ocTree.isNodeOccupied(node))
         {
            Vector3d planeNormal = new Vector3d();
            node.getNormal(planeNormal);
            boolean isNormalSet = node.isNormalSet();
            Color normalBasedColor = getNormalBasedColor(planeNormal, isNormalSet);
            if (isNormalSet)
            {
               Point3d pointOnPlane = new Point3d();
               if (node.isHitLocationSet())
                  node.getHitLocation(pointOnPlane);
               else
                  pointOnPlane.set(nodeCenter);
               intersectionPlaneBoxCalculator.setCube(boxSize, nodeCenter);
               intersectionPlaneBoxCalculator.setPlane(pointOnPlane, planeNormal);
               intersectionPlaneBoxCalculator.computeIntersections(plane);
               occupiedMeshBuilder.addPolyon(plane, normalBasedColor);
               if (SHOW_HIT_LOCATIONS)
                  occupiedMeshBuilder.addCube(0.01, pointOnPlane, DEFAULT_COLOR);
               
               

               for (int j = 0; j < plane.size(); j++)
               {
                  Point3d intersection = plane.get(j);

                  Vector3d sub = new Vector3d();
                  sub.sub(intersection, pointOnPlane);

                  Vector3d v0 = new Vector3d();
                  Vector3d v1 = new Vector3d();
                  Vector3d v3 = new Vector3d();
                  Point3d nextIntersection = plane.get((j + 1) % plane.size());
                  Point3d previousIntersection = plane.get(j == 0 ? plane.size() - 1 : j - 1);
                  v0.sub(intersection, nextIntersection);
                  v1.sub(intersection, previousIntersection);
                  v3.cross(v0, v1);
                  if (v3.dot(planeNormal) < 0.0 || Math.abs(sub.dot(planeNormal)) > 0.01)
                  {
                     System.err.println("node size: " + boxSize);
                     System.err.println("      Point3d cubeCenter = new Point3d" + nodeCenter + ";");
                     System.err.println("      Point3d pointOnPlane = new Point3d" + pointOnPlane + ";");
                     System.err.println("      Vector3d planeNormal = new Vector3d" + planeNormal + ";");
                     System.out.println();
                  }
               }
               
               
               
               
            }
            else
               occupiedMeshBuilder.addCube((float) boxSize, new Point3f(nodeCenter), normalBasedColor);
         }
         else if (SHOW_FREE_CELLS)
         {
            freeMeshBuilder.addCube((float) boxSize, new Point3f(nodeCenter));
         }
      }

      MeshView occupiedMeshView = new MeshView();
      occupiedMeshView.setMesh(occupiedMeshBuilder.generateMesh());
      occupiedMeshView.setMaterial(occupiedMeshBuilder.generateMaterial());
      occupiedMeshView.setMouseTransparent(true);
      view3dFactory.addNodeToView(occupiedMeshView);

      if (SHOW_FREE_CELLS)
      {
         MeshView freeMeshView = new MeshView();
         freeMeshView.setMesh(freeMeshBuilder.generateMesh());
         PhongMaterial material = new PhongMaterial();
         material.setDiffuseColor(FREE_COLOR);
         freeMeshView.setMaterial(material);
         view3dFactory.addNodeToView(freeMeshView);
      }

      if (SHOW_POINT_CLOUD)
      {
         for (int i = 0; i < pointcloud.getNumberOfPoints(); i++)
         {
            Sphere sphere = new Sphere(0.0025);
            sphere.setTranslateX(pointcloud.getPoint(i).getX());
            sphere.setTranslateY(pointcloud.getPoint(i).getY());
            sphere.setTranslateZ(pointcloud.getPoint(i).getZ());
            view3dFactory.addNodeToView(sphere);
         }
      }

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   private static final Color DEFAULT_COLOR = Color.DARKCYAN;

   public Color getNormalBasedColor(Vector3d normal, boolean isNormalSet)
   {
      Color color = DEFAULT_COLOR;

      if (isNormalSet)
      {
         Vector3d zUp = new Vector3d(0.0, 0.0, 1.0);
         normal.normalize();
         double angle = Math.abs(zUp.dot(normal));
         double hue = 120.0 * angle;
         color = Color.hsb(hue, 1.0, 1.0);
      }
      return color;
   }

   public static void main(String[] args)
   {

      //      new OcTreeVisualizer();

      Application.launch(args);
   }
}
