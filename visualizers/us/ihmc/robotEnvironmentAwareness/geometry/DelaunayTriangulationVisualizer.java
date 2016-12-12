package us.ihmc.robotEnvironmentAwareness.geometry;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.ImmutablePair;

import com.vividsolutions.jts.triangulate.quadedge.QuadEdge;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdgeTriangle;
import com.vividsolutions.jts.triangulate.quadedge.Vertex;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.collections.ObservableList;
import javafx.event.EventHandler;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullFactoryIntermediateVariables;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullFactoryResult;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataImporter;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.lists.ListWrappingIndexTools;

public class DelaunayTriangulationVisualizer extends Application
{
   private static final boolean VISUALIZE_POINT_CLOUD = true;
   private static final boolean VISUALIZE_DELAUNAY_TRIANGULATION = false;
   private static final boolean VISUALIZE_CONCAVE_HULL = true;
   private static final boolean VISUALIZE_BORDER_EDGES = false;
   private static final boolean VISUALIZE_OUTER_TRIANGLES = false;
   private static final boolean VISUALIZE_PRIORITY_QUEUE = false;
   private static final boolean VISUALIZE_CONVEX_DECOMPOSITION = false;
   private static final boolean VISUALIZE_BORDER_VERTICES = false;
   private static final boolean VISUALIZE_CONCAVE_POCKETS = true;

   private static final boolean FILTER_CONCAVE_HULLS = false;

   private final Random random = new Random(54645L);
   private final PolygonizerParameters parameters = new PolygonizerParameters();

   public DelaunayTriangulationVisualizer() throws IOException
   {
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle("OcTree Visualizer");

//      PlanarRegionSegmentationDataImporter dataImporter = new PlanarRegionSegmentationDataImporter(new File("Data/20161210_185643_PlanarRegionSegmentation_Atlas_CB"));
      PlanarRegionSegmentationDataImporter dataImporter = PlanarRegionSegmentationDataImporter.createImporterWithFileChooser(primaryStage);
      if (dataImporter == null)
         Platform.exit();
      dataImporter.loadPlanarRegionSegmentationData();
      List<PlanarRegionSegmentationMessage> planarRegionSegmentationData = dataImporter.getPlanarRegionSegmentationData();

      View3DFactory view3dFactory = new View3DFactory(600, 400);
      view3dFactory.addCameraController();
      view3dFactory.addWorldCoordinateSystem(0.3);

      Point3d average = computeAverage(planarRegionSegmentationData);
      average.negate();

      Map<Node, Integer> nodeToRegionId = new HashMap<>();

      for (PlanarRegionSegmentationMessage planarRegionSegmentationMessage : planarRegionSegmentationData)
      {
         Node regionGraphics = createRegionGraphics(planarRegionSegmentationMessage);
         regionGraphics.setManaged(false);
         translateNode(regionGraphics, average);
         nodeToRegionId.put(regionGraphics, planarRegionSegmentationMessage.getRegionId());
         view3dFactory.addNodeToView(regionGraphics);
      }

      Scene scene = view3dFactory.getScene();
      scene.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            if (!event.isShiftDown())
               return;
            Node intersectedNode = event.getPickResult().getIntersectedNode();

            Integer regionId = nodeToRegionId.get(intersectedNode);

            while (regionId == null && intersectedNode.getParent() != null)
            {
               intersectedNode = intersectedNode.getParent();
               regionId = nodeToRegionId.get(intersectedNode);
            }

            if (regionId != null)
            {
               System.out.println("Region picked: " + regionId);
               for (Entry<Node, Integer> nodeAndId : nodeToRegionId.entrySet())
               {
                  if (nodeAndId.getValue() != regionId)
                     nodeAndId.getKey().setVisible(false);
               }
            }
         }
      });

      primaryStage.addEventHandler(KeyEvent.KEY_PRESSED, event -> {
         if (event.getCode() == KeyCode.F5)
            nodeToRegionId.keySet().stream().forEach(node -> node.setVisible(true));
      });

      primaryStage.setScene(scene);
      primaryStage.show();
   }

   private void translateNode(Node nodeToTranslate, Tuple3d translation)
   {
      nodeToTranslate.setTranslateX(nodeToTranslate.getTranslateX() + translation.getX());
      nodeToTranslate.setTranslateY(nodeToTranslate.getTranslateY() + translation.getY());
      nodeToTranslate.setTranslateZ(nodeToTranslate.getTranslateZ() + translation.getZ());
   }

   private Point3d computeAverage(List<PlanarRegionSegmentationMessage> planarRegionSegmentationMessages)
   {
      PointMean average = new PointMean();

      for (PlanarRegionSegmentationMessage planarRegionSegmentationMessage : planarRegionSegmentationMessages)
         Arrays.stream(planarRegionSegmentationMessage.getHitLocations()).forEach(average::update);

      return average;
   }

   private Node createRegionGraphics(PlanarRegionSegmentationMessage planarRegionSegmentationMessage)
   {
      Group regionGroup = new Group();
      ObservableList<Node> children = regionGroup.getChildren();

      List<Point2d> pointsInPlane = PolygonizerTools.extractPointsInPlane(planarRegionSegmentationMessage);
      ConcaveHullFactoryResult concaveHullFactoryResult = SimpleConcaveHullFactory.createConcaveHull(pointsInPlane, parameters.getConcaveHullThreshold());

      if (VISUALIZE_CONCAVE_HULL)
         children.add(createConcaveHullGraphics(planarRegionSegmentationMessage, concaveHullFactoryResult));
      if (VISUALIZE_POINT_CLOUD)
         children.add(createRegionPointCloudGraphics(planarRegionSegmentationMessage));
      if (VISUALIZE_OUTER_TRIANGLES)
         children.add(createOuterTrianglesGraphics(planarRegionSegmentationMessage, concaveHullFactoryResult));
      if (VISUALIZE_DELAUNAY_TRIANGULATION)
         children.add(createDelaunayTriangulationGraphics(planarRegionSegmentationMessage, concaveHullFactoryResult));
      if (VISUALIZE_BORDER_EDGES)
         children.add(createBorderEdgesGraphics(planarRegionSegmentationMessage, concaveHullFactoryResult));
      if (VISUALIZE_CONVEX_DECOMPOSITION)
         children.add(createConvexDecompositionGraphics(planarRegionSegmentationMessage, concaveHullFactoryResult));
      if (VISUALIZE_PRIORITY_QUEUE)
         children.add(createPriorityQueueGraphics(planarRegionSegmentationMessage, concaveHullFactoryResult));
      if (VISUALIZE_BORDER_VERTICES)
         children.add(createBorderVerticesGraphics(planarRegionSegmentationMessage, concaveHullFactoryResult));
      if (VISUALIZE_CONCAVE_POCKETS)
         children.add(createConcavePocketsGraphics(planarRegionSegmentationMessage, concaveHullFactoryResult));
      return regionGroup;
   }

   private Node createConcavePocketsGraphics(PlanarRegionSegmentationMessage planarRegionSegmentationMessage, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
      ConcaveHullCollection concaveHullCollection = concaveHullFactoryResult.getConcaveHullCollection();

      for (ConcaveHull concaveHull : concaveHullCollection)
      {
         Set<ConcaveHullPocket> pockets = concaveHull.findConcaveHullPockets(0.5 * parameters.getDepthThreshold());
         Point3d planeOrigin = new Point3d(planarRegionSegmentationMessage.getOrigin());
         Quat4d planeOrientation = PolygonizerTools.getRotationBasedOnNormal(planarRegionSegmentationMessage.getNormal());
         RigidBodyTransform transform = new RigidBodyTransform(planeOrientation, planeOrigin);

         for (ConcaveHullPocket pocket : pockets)
         {
            List<Point2d> pocketVertices = ListWrappingIndexTools.subListInclusive(pocket.getStartBridgeIndex(), pocket.getEndBridgeIndex(),
                  concaveHull.getConcaveHullVertices());
            Point2d average = new Point2d();
            average.interpolate(pocket.getStartBridgeVertex(), pocket.getEndBridgeVertex(), 0.5);
            pocketVertices.add(0, average);
            ConcaveHullTools.ensureClockwiseOrdering(pocketVertices);
            meshBuilder.addPolygon(transform, pocketVertices);
         }
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(OcTreeMeshBuilder.getRegionColor(planarRegionSegmentationMessage.getRegionId())));
      return meshView;
   }

   private Node createBorderEdgesGraphics(PlanarRegionSegmentationMessage planarRegionSegmentationMessage, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      int regionId = planarRegionSegmentationMessage.getRegionId();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(16));
      Point3d planeOrigin = new Point3d(planarRegionSegmentationMessage.getOrigin());
      Quat4d planeOrientation = PolygonizerTools.getRotationBasedOnNormal(planarRegionSegmentationMessage.getNormal());
      Color regionColor = OcTreeMeshBuilder.getRegionColor(regionId);

      for (ConcaveHullFactoryIntermediateVariables intermediateVariables : concaveHullFactoryResult.getIntermediateVariables())
      {
         Set<QuadEdge> borderEdges = intermediateVariables.getBorderEdges();

         for (QuadEdge edge : borderEdges)
         {
            Point3d dest = PolygonizerTools.toPointInWorld(edge.dest().getX(), edge.dest().getY(), planeOrigin, planeOrientation);
            Point3d orig = PolygonizerTools.toPointInWorld(edge.orig().getX(), edge.orig().getY(), planeOrigin, planeOrientation);
            boolean isEdgeTooLong = dest.distance(orig) > parameters.getConcaveHullThreshold();
            Color lineColor = Color.hsb(regionColor.getHue(), regionColor.getSaturation(), isEdgeTooLong ? 0.25 : regionColor.getBrightness());
            meshBuilder.addLine(dest, orig, 0.0015, lineColor);
         }
      }
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      meshView.setMouseTransparent(true);
      return meshView;
   }

   private Node createRegionPointCloudGraphics(PlanarRegionSegmentationMessage planarRegionSegmentationMessage)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      for (Point3f hitLocation : planarRegionSegmentationMessage.getHitLocations())
         meshBuilder.addTetrahedron(0.0025f, hitLocation);
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(OcTreeMeshBuilder.getRegionColor(planarRegionSegmentationMessage.getRegionId())));
      meshView.setMouseTransparent(true);
      return meshView;
   }
   
   private Node createBorderVerticesGraphics(PlanarRegionSegmentationMessage planarRegionSegmentationMessage, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
      Point3d planeOrigin = new Point3d(planarRegionSegmentationMessage.getOrigin());
      Quat4d planeOrientation = PolygonizerTools.getRotationBasedOnNormal(planarRegionSegmentationMessage.getNormal());

      for (ConcaveHullFactoryIntermediateVariables intermediateVariables : concaveHullFactoryResult.getIntermediateVariables())
      {
         for (Vertex vertex2d : intermediateVariables.getBorderVertices())
         {
            Point3d vertex3d = PolygonizerTools.toPointInWorld(vertex2d.getX(), vertex2d.getY(), planeOrigin, planeOrientation);
            meshBuilder.addSphere(0.003, vertex3d);
         }
      }
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(OcTreeMeshBuilder.getRegionColor(planarRegionSegmentationMessage.getRegionId())));
      meshView.setMouseTransparent(true);
      return meshView;
   }

   private Node createConcaveHullGraphics(PlanarRegionSegmentationMessage planarRegionSegmentationMessage, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      int regionId = planarRegionSegmentationMessage.getRegionId();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(16));
      Point3d planeOrigin = new Point3d(planarRegionSegmentationMessage.getOrigin());
      Vector3d planeNormal = new Vector3d(planarRegionSegmentationMessage.getNormal());
      ConcaveHullCollection concaveHullCollection = concaveHullFactoryResult.getConcaveHullCollection();

      for (ConcaveHull concaveHull : concaveHullCollection)
      {

         if (FILTER_CONCAVE_HULLS)
         {
            double shallowAngleThreshold = parameters.getShallowAngleThreshold();
            double peakAngleThreshold = parameters.getPeakAngleThreshold();
            double lengthThreshold = parameters.getLengthThreshold();

            for (int i = 0; i < 5; i++)
            {
               ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHull);
               ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHull);
            }
         }
         Color regionColor = OcTreeMeshBuilder.getRegionColor(regionId);

         List<Point3d> concaveHullVertices = concaveHull.toVerticesInWorld(planeOrigin, planeNormal);

         for (int vertexIndex = 0; vertexIndex < concaveHullVertices.size(); vertexIndex++)
         {
            Point3d vertex = concaveHullVertices.get(vertexIndex);
            Point3d nextVertex = ListWrappingIndexTools.getNext(vertexIndex, concaveHullVertices);
            boolean isEdgeTooLong = vertex.distance(nextVertex) > parameters.getConcaveHullThreshold();
            Color lineColor = Color.hsb(regionColor.getHue(), regionColor.getSaturation(), isEdgeTooLong ? 0.25 : regionColor.getBrightness());
            meshBuilder.addLine(vertex, nextVertex, 0.0015, lineColor);
         }
      }
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      meshView.setMouseTransparent(true);
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

   private Node createOuterTrianglesGraphics(PlanarRegionSegmentationMessage planarRegionSegmentationMessage, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(512));

      Point3d planeOrigin = new Point3d(planarRegionSegmentationMessage.getOrigin());
      Vector3d planeNormal = new Vector3d(planarRegionSegmentationMessage.getNormal());

      for (ConcaveHullFactoryIntermediateVariables intermediateVariables : concaveHullFactoryResult.getIntermediateVariables())
      {
         Set<QuadEdgeTriangle> outerTriangles = intermediateVariables.getOuterTriangles();

         for (QuadEdgeTriangle triangle : outerTriangles)
         {
            List<Point2d> triangleVerticesLocal = Arrays.stream(triangle.getVertices()).map(v -> new Point2d(v.getX(), v.getY())).collect(Collectors.toList());
            List<Point3d> triangleVerticesWorld = PolygonizerTools.toPointsInWorld(triangleVerticesLocal, planeOrigin, planeNormal);
            double hue = 360.0 * random.nextDouble();
            double saturation = 0.8 * random.nextDouble() + 0.1;
            double brightness = 0.9;

            meshBuilder.addPolyon(triangleVerticesWorld, Color.hsb(hue, saturation, brightness));
         }
      }

      MeshView trianglesMeshView = new MeshView(meshBuilder.generateMesh());
      trianglesMeshView.setMaterial(meshBuilder.generateMaterial());
      return trianglesMeshView;
   }

   private Node createPriorityQueueGraphics(PlanarRegionSegmentationMessage planarRegionSegmentationMessage, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(512));

      Point3d planeOrigin = new Point3d(planarRegionSegmentationMessage.getOrigin());
      Vector3d planeNormal = new Vector3d(planarRegionSegmentationMessage.getNormal());
      Quat4d planeOrientation = PolygonizerTools.getRotationBasedOnNormal(planeNormal);

      Color regionColor = OcTreeMeshBuilder.getRegionColor(planarRegionSegmentationMessage.getRegionId());

      for (ConcaveHullFactoryIntermediateVariables intermediateVariables : concaveHullFactoryResult.getIntermediateVariables())
      {
         PriorityQueue<ImmutablePair<QuadEdge, QuadEdgeTriangle>> queue = intermediateVariables.getSortedByLengthQueue();

         for (ImmutablePair<QuadEdge, QuadEdgeTriangle> edgeAndTriangle : queue)
         {
            QuadEdge edge = edgeAndTriangle.getLeft();
            Point3d dest = PolygonizerTools.toPointInWorld(edge.dest().getX(), edge.dest().getY(), planeOrigin, planeOrientation);
            Point3d orig = PolygonizerTools.toPointInWorld(edge.orig().getX(), edge.orig().getY(), planeOrigin, planeOrientation);
            boolean isEdgeTooLong = dest.distance(orig) > parameters.getConcaveHullThreshold();
            Color lineColor = Color.hsb(regionColor.getHue(), regionColor.getSaturation(), isEdgeTooLong ? 0.25 : regionColor.getBrightness());
            meshBuilder.addLine(dest, orig, 0.0015, lineColor);

            QuadEdgeTriangle triangle = edgeAndTriangle.getRight();
            List<Point2d> triangleVerticesLocal = Arrays.stream(triangle.getVertices()).map(v -> new Point2d(v.getX(), v.getY())).collect(Collectors.toList());
            List<Point3d> triangleVerticesWorld = PolygonizerTools.toPointsInWorld(triangleVerticesLocal, planeOrigin, planeNormal);
            double hue = 360.0 * random.nextDouble();
            double saturation = 0.8 * random.nextDouble() + 0.1;
            double brightness = 0.9;

            meshBuilder.addPolyon(triangleVerticesWorld, Color.hsb(hue, saturation, brightness));
         }
      }

      MeshView trianglesMeshView = new MeshView(meshBuilder.generateMesh());
      trianglesMeshView.setMaterial(meshBuilder.generateMaterial());
      return trianglesMeshView;
   }

   private Node createConvexDecompositionGraphics(PlanarRegionSegmentationMessage planarRegionSegmentationMessage,
         ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      ConcaveHullCollection concaveHullCollection = concaveHullFactoryResult.getConcaveHullCollection();
      double depthThreshold = parameters.getDepthThreshold();
      List<ConvexPolygon2d> convexPolygons = new ArrayList<>();
      ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHullCollection, depthThreshold, convexPolygons);

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
