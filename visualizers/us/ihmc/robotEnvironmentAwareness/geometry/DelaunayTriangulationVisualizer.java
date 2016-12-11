package us.ihmc.robotEnvironmentAwareness.geometry;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import com.vividsolutions.jts.triangulate.quadedge.QuadEdgeTriangle;

import javafx.application.Application;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullFactoryResult;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataImporter;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class DelaunayTriangulationVisualizer extends Application
{
   private static final boolean VISUALIZE_POINT_CLOUD = true;
   private static final boolean VISUALIZE_DELAUNAY_TRIANGULATION = true;
   private static final boolean VISUALIZE_CONVEX_DECOMPOSITION = false;

   private final Random random = new Random(54645L);

   public DelaunayTriangulationVisualizer() throws IOException
   {
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle("OcTree Visualizer");

      PlanarRegionSegmentationDataImporter dataImporter = PlanarRegionSegmentationDataImporter.createImporterWithFileChooser(primaryStage);
      dataImporter.loadPlanarRegionSegmentationData();
      List<PlanarRegionSegmentationMessage> planarRegionSegmentationData = dataImporter.getPlanarRegionSegmentationData();

      View3DFactory view3dFactory = new View3DFactory(600, 400);
      view3dFactory.addCameraController();
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.setRootMouseTransparent(true);

      for (PlanarRegionSegmentationMessage planarRegionSegmentationMessage : planarRegionSegmentationData)
         view3dFactory.addNodeToView(createRegionGraphics(planarRegionSegmentationMessage));

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   private Node createRegionGraphics(PlanarRegionSegmentationMessage planarRegionSegmentationMessage)
   {
      Group regionGroup = new Group();

      List<Point2d> pointsInPlane = PolygonizerTools.extractPointsInPlane(planarRegionSegmentationMessage);
      ConcaveHullFactoryResult concaveHullFactoryResult = SimpleConcaveHullFactory.createConcaveHull(pointsInPlane, 0.05);

      regionGroup.getChildren().add(createConcaveHullGraphics(planarRegionSegmentationMessage, concaveHullFactoryResult));
      if (VISUALIZE_POINT_CLOUD)
         regionGroup.getChildren().add(createRegionPointCloudGraphics(planarRegionSegmentationMessage));
      if (VISUALIZE_DELAUNAY_TRIANGULATION)
         regionGroup.getChildren().add(createDelaunayTriangulationGraphics(planarRegionSegmentationMessage, concaveHullFactoryResult));
      if (VISUALIZE_CONVEX_DECOMPOSITION)
         regionGroup.getChildren().add(createConvexDecompositionGraphics(planarRegionSegmentationMessage, concaveHullFactoryResult));
      return regionGroup;
   }

   private Node createRegionPointCloudGraphics(PlanarRegionSegmentationMessage planarRegionSegmentationMessage)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      for (Point3f hitLocation : planarRegionSegmentationMessage.getHitLocations())
         meshBuilder.addTetrahedron(0.0075f, hitLocation);
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(OcTreeMeshBuilder.getRegionColor(planarRegionSegmentationMessage.getRegionId())));
      return meshView;
   }

   private Node createConcaveHullGraphics(PlanarRegionSegmentationMessage planarRegionSegmentationMessage, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      int regionId = planarRegionSegmentationMessage.getRegionId();
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
      Point3d planeOrigin = new Point3d(planarRegionSegmentationMessage.getOrigin());
      Vector3d planeNormal = new Vector3d(planarRegionSegmentationMessage.getNormal());
      List<Point3d> concaveHullVertices = PolygonizerTools.toPointsInWorld(concaveHullFactoryResult.getOrderedConcaveHullVertices(), planeOrigin, planeNormal);
      meshBuilder.addMultiLine(concaveHullVertices, 0.005, true);
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(OcTreeMeshBuilder.getRegionColor(regionId)));
      return meshView;
   }

   private Node createDelaunayTriangulationGraphics(PlanarRegionSegmentationMessage planarRegionSegmentationMessage,
         ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(512));

      Point3d planeOrigin = new Point3d(planarRegionSegmentationMessage.getOrigin());
      Vector3d planeNormal = new Vector3d(planarRegionSegmentationMessage.getNormal());

      List<QuadEdgeTriangle> allTriangles = concaveHullFactoryResult.getAllTriangles();

      for (QuadEdgeTriangle triangle : allTriangles)
      {
         List<Point2d> triangleVerticesLocal = Arrays.stream(triangle.getVertices()).map(v -> new Point2d(v.getX(), v.getY())).collect(Collectors.toList());
         List<Point3d> triangleVerticesWorld = PolygonizerTools.toPointsInWorld(triangleVerticesLocal, planeOrigin, planeNormal);
         double hue = 360.0 * random.nextDouble();
         double saturation = 0.8 * random.nextDouble() + 0.1;
         double brightness = 0.9;

         meshBuilder.addPolyon(triangleVerticesWorld, Color.hsb(hue, saturation, brightness));
      }

      MeshView trianglesMeshView = new MeshView(meshBuilder.generateMesh());
      trianglesMeshView.setMaterial(meshBuilder.generateMaterial());
      return trianglesMeshView;
   }

   private Node createConvexDecompositionGraphics(PlanarRegionSegmentationMessage planarRegionSegmentationMessage,
         ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      List<Point2d> concaveHullVertices = concaveHullFactoryResult.getOrderedConcaveHullVertices();
      double depthThreshold = 0.05;
      List<ConvexPolygon2d> convexPolygons = new ArrayList<>();
      ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHullVertices, depthThreshold, convexPolygons);

      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(64));

      int regionId = planarRegionSegmentationMessage.getRegionId();
      Quat4d regionOrientation = PolygonizerTools.getRotationBasedOnNormal(planarRegionSegmentationMessage.getNormal());
      Point3d regionOrigin = new Point3d(planarRegionSegmentationMessage.getOrigin());
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform(regionOrientation, regionOrigin);
      Color regionColor = OcTreeMeshBuilder.getRegionColor(regionId);

      for (int i = 0; i < convexPolygons.size(); i++)
      {
         ConvexPolygon2d convexPolygon = convexPolygons.get(i);
         Color color = Color.hsb(regionColor.getHue(), 0.9, 0.5 + 0.5 * ((double) i / (double) convexPolygons.size()));
         meshBuilder.addPolygon(rigidBodyTransform, convexPolygon, color);
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      return meshView;
   }

   public static void main(String[] args)
   {
      Application.launch();
   }
}
