package us.ihmc.robotEnvironmentAwareness.geometry;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.Pair;

import com.vividsolutions.jts.triangulate.quadedge.QuadEdge;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdgeTriangle;
import com.vividsolutions.jts.triangulate.quadedge.Vertex;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.beans.Observable;
import javafx.beans.property.DoubleProperty;
import javafx.collections.ObservableList;
import javafx.event.EventHandler;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.SubScene;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullFactoryResult;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullVariables;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationRawDataImporter;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.lists.ListWrappingIndexTools;

public class PolygonizerVisualizer extends Application
{
   private static final boolean VISUALIZE_POINT_CLOUD = true;
   private static final boolean VISUALIZE_DELAUNAY_TRIANGULATION = true;
   private static final boolean VISUALIZE_CONCAVE_HULL = false;
   private static final boolean VISUALIZE_BORDER_EDGES = true;
   private static final boolean VISUALIZE_BORDER_TRIANGLES = false;
   private static final boolean VISUALIZE_PRIORITY_QUEUE = false;
   private static final boolean VISUALIZE_CONVEX_DECOMPOSITION = false;
   private static final boolean VISUALIZE_BORDER_VERTICES = false;
   private static final boolean VISUALIZE_CONCAVE_POCKETS = false;
   private static final boolean VISUALIZE_ORDERED_BORDER_EDGES = true;

   private static final double scaleX = 1.0;
   private static final double scaleY = 1.0;

   private static final boolean FILTER_CONCAVE_HULLS = false;

   private static final int[] onlyRegionWithId = {};

   private File defaultFile = null;// new File("DataThrowingException/20161213_220649_PlanarRegionSegmentation");

   private final Random random = new Random(54645L);
   private final ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();

   public PolygonizerVisualizer() throws IOException
   {
      parameters.setEdgeLengthThreshold(0.05);
      //      parameters.setAllowSplittingConcaveHull(false);
      //      parameters.setRemoveAllTrianglesWithTwoBorderEdges(false);
//      parameters.setMaxNumberOfIterations(0);
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle(getClass().getSimpleName());

      PlanarRegionSegmentationRawDataImporter dataImporter;
      if (defaultFile != null)
         dataImporter = new PlanarRegionSegmentationRawDataImporter(defaultFile);
      else
         dataImporter = PlanarRegionSegmentationRawDataImporter.createImporterWithFileChooser(primaryStage);
      if (dataImporter == null)
         Platform.exit();
      dataImporter.loadPlanarRegionSegmentationData();
      List<PlanarRegionSegmentationRawData> regionsRawData = dataImporter.getPlanarRegionSegmentationRawData();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.enableRequestFocusOnMouseClicked();
      SubScene scene = view3dFactory.getSubScene();

      Set<Integer> regionIdFilterSet = new HashSet<>();
      Arrays.stream(onlyRegionWithId).forEach(regionIdFilterSet::add);

      if (regionIdFilterSet.size() == 1 || regionsRawData.size() == 1)
      {
         PlanarRegionSegmentationRawData rawData;
         if (regionsRawData.size() == 1)
            rawData = regionsRawData.get(0);
         else
            rawData = regionsRawData.stream().filter(region -> regionIdFilterSet.contains(region.getRegionId())).findFirst().get();

         RigidBodyTransform transform = rawData.getTransformFromLocalToWorld();
         transform.invert();

         Node regionGraphics = createRegionGraphics(rawData);
         transformNode(regionGraphics, transform);
         view3dFactory.addNodeToView(regionGraphics);
      }
      else
      {
         Map<Node, Integer> nodeToRegionId = new HashMap<>();

         Point3d average = computeAverage(regionsRawData, regionIdFilterSet);
         average.negate();

         for (PlanarRegionSegmentationRawData rawData : regionsRawData)
         {
            if (regionIdFilterSet.isEmpty() || regionIdFilterSet.contains(rawData.getRegionId()))
            {
               Node regionGraphics = createRegionGraphics(rawData);
               regionGraphics.setManaged(false);
               translateNode(regionGraphics, average);
               nodeToRegionId.put(regionGraphics, rawData.getRegionId());
               view3dFactory.addNodeToView(regionGraphics);
            }
         }

         scene.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
         {
            @Override
            public void handle(MouseEvent event)
            {
               if (!event.isAltDown())
                  return;
               Node intersectedNode = event.getPickResult().getIntersectedNode();
               if (intersectedNode == null)
                  return;

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
      }

      HBox coordinatesHBox = setupFocusCoordinatesViz(cameraController);
      BorderPane mainPane = new BorderPane();
      view3dFactory.bindSubSceneSizeToPaneSize(mainPane);
      mainPane.setCenter(scene);
      mainPane.setTop(coordinatesHBox);

      primaryStage.setScene(new Scene(mainPane, 800, 400, true));
      primaryStage.show();
   }

   private HBox setupFocusCoordinatesViz(FocusBasedCameraMouseEventHandler cameraController)
   {
      HBox coordinatesHBox = new HBox();

      DoubleProperty xProperty = cameraController.getTranslate().xProperty();
      DoubleProperty yProperty = cameraController.getTranslate().yProperty();
      DoubleProperty zProperty = cameraController.getTranslate().zProperty();

      Label xFocusLabel = new Label("x focus: ");
      Label yFocusLabel = new Label("y focus: ");
      Label zFocusLabel = new Label("z focus: ");

      TextField xFocus = new TextField("blop");
      TextField yFocus = new TextField("blop");
      TextField zFocus = new TextField("blop");

      xFocus.setEditable(false);
      yFocus.setEditable(false);
      zFocus.setEditable(false);

      xProperty.addListener((Observable o) -> xFocus.setText(xProperty.getValue().toString()));
      yProperty.addListener((Observable o) -> yFocus.setText(yProperty.getValue().toString()));
      zProperty.addListener((Observable o) -> zFocus.setText(zProperty.getValue().toString()));

      coordinatesHBox.getChildren().addAll(xFocusLabel, xFocus, yFocusLabel, yFocus, zFocusLabel, zFocus);
      return coordinatesHBox;
   }

   public static void translateNode(Node nodeToTranslate, Tuple3d translation)
   {
      nodeToTranslate.setTranslateX(nodeToTranslate.getTranslateX() + translation.getX());
      nodeToTranslate.setTranslateY(nodeToTranslate.getTranslateY() + translation.getY());
      nodeToTranslate.setTranslateZ(nodeToTranslate.getTranslateZ() + translation.getZ());
   }

   public static void transformNode(Node nodeToTransform, RigidBodyTransform transform)
   {
      nodeToTransform.getTransforms().add(JavaFXTools.convertRigidBodyTransformToAffine(transform));
   }

   public static Point3d computeAverage(List<PlanarRegionSegmentationRawData> regionsRawData, Set<Integer> regionIdFilterSet)
   {
      PointMean average = new PointMean();

      for (PlanarRegionSegmentationRawData rawData : regionsRawData)
      {
         if (regionIdFilterSet.isEmpty() || regionIdFilterSet.contains(rawData.getRegionId()))
            rawData.stream().forEach(average::update);
      }

      return average;
   }

   private Node createRegionGraphics(PlanarRegionSegmentationRawData rawData)
   {
      Group regionGroup = new Group();
      ObservableList<Node> children = regionGroup.getChildren();

      List<Point2d> pointsInPlane = rawData.getPointCloudInPlane();
      ConcaveHullFactoryResult concaveHullFactoryResult = SimpleConcaveHullFactory.createConcaveHull(pointsInPlane, parameters);

      if (VISUALIZE_CONCAVE_HULL)
         children.add(createConcaveHullGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_POINT_CLOUD)
         children.add(createRegionPointCloudGraphics(rawData));
      if (VISUALIZE_BORDER_TRIANGLES)
         children.add(createBorderTrianglesGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_DELAUNAY_TRIANGULATION)
         children.add(createDelaunayTriangulationGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_BORDER_EDGES)
         children.add(createBorderEdgesGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_CONVEX_DECOMPOSITION)
         children.add(createConvexDecompositionGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_PRIORITY_QUEUE)
         children.add(createPriorityQueueGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_BORDER_VERTICES)
         children.add(createBorderVerticesGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_CONCAVE_POCKETS)
         children.add(createConcavePocketsGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_ORDERED_BORDER_EDGES)
         children.add(createOrderedBorderEdgesGraphics(rawData, concaveHullFactoryResult));

      return regionGroup;
   }

   private Node createConcavePocketsGraphics(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
      ConcaveHullCollection concaveHullCollection = concaveHullFactoryResult.getConcaveHullCollection();
      RigidBodyTransform transform = rawData.getTransformFromLocalToWorld();

      for (ConcaveHull concaveHull : concaveHullCollection)
      {
         Set<ConcaveHullPocket> pockets = concaveHull.findConcaveHullPockets(polygonizerParameters.getDepthThreshold());

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
      meshView.setMaterial(new PhongMaterial(OcTreeMeshBuilder.getRegionColor(rawData.getRegionId())));
      return meshView;
   }

   private Node createBorderEdgesGraphics(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      int regionId = rawData.getRegionId();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(16));
      Point3d planeOrigin = rawData.getOrigin();
      Quat4d planeOrientation = rawData.getOrientation();
      Color regionColor = OcTreeMeshBuilder.getRegionColor(regionId);

      for (ConcaveHullVariables intermediateVariables : concaveHullFactoryResult.getIntermediateVariables())
      {
         Set<QuadEdge> borderEdges = intermediateVariables.getBorderEdges();

         for (QuadEdge edge : borderEdges)
         {
            Point3d dest = PolygonizerTools.toPointInWorld(edge.dest().getX(), edge.dest().getY(), planeOrigin, planeOrientation);
            Point3d orig = PolygonizerTools.toPointInWorld(edge.orig().getX(), edge.orig().getY(), planeOrigin, planeOrientation);
            boolean isEdgeTooLong = dest.distance(orig) > parameters.getEdgeLengthThreshold();
            Color lineColor = Color.hsb(regionColor.getHue(), regionColor.getSaturation(), isEdgeTooLong ? 0.25 : regionColor.getBrightness());
            meshBuilder.addLine(dest, orig, 0.0015, lineColor);
         }
      }
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      meshView.setMouseTransparent(true);
      return meshView;
   }

   private Node createRegionPointCloudGraphics(PlanarRegionSegmentationRawData rawData)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      rawData.stream().forEach(point -> meshBuilder.addTetrahedron(0.0025, point));
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(OcTreeMeshBuilder.getRegionColor(rawData.getRegionId())));
      meshView.setMouseTransparent(true);
      return meshView;
   }

   private Node createBorderVerticesGraphics(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
      Point3d planeOrigin = rawData.getOrigin();
      Quat4d planeOrientation = rawData.getOrientation();

      for (ConcaveHullVariables intermediateVariables : concaveHullFactoryResult.getIntermediateVariables())
      {
         for (Vertex vertex2d : intermediateVariables.getBorderVertices())
         {
            Point3d vertex3d = PolygonizerTools.toPointInWorld(vertex2d.getX(), vertex2d.getY(), planeOrigin, planeOrientation);
            meshBuilder.addSphere(0.003, vertex3d);
         }
      }
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(OcTreeMeshBuilder.getRegionColor(rawData.getRegionId())));
      meshView.setMouseTransparent(true);
      return meshView;
   }

   private Node createConcaveHullGraphics(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      int regionId = rawData.getRegionId();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(16));
      Point3d planeOrigin = rawData.getOrigin();
      Vector3d planeNormal = rawData.getNormal();
      ConcaveHullCollection concaveHullCollection = concaveHullFactoryResult.getConcaveHullCollection();

      for (ConcaveHull concaveHull : concaveHullCollection)
      {

         if (FILTER_CONCAVE_HULLS)
         {
            double shallowAngleThreshold = polygonizerParameters.getShallowAngleThreshold();
            double peakAngleThreshold = polygonizerParameters.getPeakAngleThreshold();
            double lengthThreshold = polygonizerParameters.getLengthThreshold();

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
            boolean isEdgeTooLong = vertex.distance(nextVertex) > parameters.getEdgeLengthThreshold();
            Color lineColor = Color.hsb(regionColor.getHue(), regionColor.getSaturation(), isEdgeTooLong ? 0.25 : regionColor.getBrightness());
            meshBuilder.addLine(vertex, nextVertex, 0.0015, lineColor);
         }
      }
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      meshView.setMouseTransparent(true);
      return meshView;
   }

   private Node createDelaunayTriangulationGraphics(PlanarRegionSegmentationRawData rawData,
         ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(512));

      Point3d planeOrigin = rawData.getOrigin();
      Vector3d planeNormal = rawData.getNormal();

      List<QuadEdgeTriangle> allTriangles = concaveHullFactoryResult.getAllTriangles();

      for (QuadEdgeTriangle triangle : allTriangles)
      {
         List<Point2d> triangleVerticesLocal = Arrays.stream(triangle.getVertices()).map(v -> new Point2d(v.getX(), v.getY())).collect(Collectors.toList());
         triangleVerticesLocal.forEach(vertex -> {
            vertex.x *= scaleX;
            vertex.y *= scaleY;
         });
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

   private Node createBorderTrianglesGraphics(PlanarRegionSegmentationRawData rawData,
         ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(512));

      Point3d planeOrigin = rawData.getOrigin();
      Vector3d planeNormal = rawData.getNormal();

      for (ConcaveHullVariables intermediateVariables : concaveHullFactoryResult.getIntermediateVariables())
      {
         Set<QuadEdgeTriangle> borderTriangles = intermediateVariables.getBorderTriangles();

         for (QuadEdgeTriangle borderTriangle : borderTriangles)
         {
            List<Point2d> triangleVerticesLocal = Arrays.stream(borderTriangle.getVertices()).map(v -> new Point2d(v.getX(), v.getY()))
                  .collect(Collectors.toList());
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

   private Node createPriorityQueueGraphics(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(512));

      Point3d planeOrigin = rawData.getOrigin();
      Quat4d planeOrientation = rawData.getOrientation();

      Color regionColor = OcTreeMeshBuilder.getRegionColor(rawData.getRegionId());

      for (ConcaveHullVariables intermediateVariables : concaveHullFactoryResult.getIntermediateVariables())
      {
         PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> queue = intermediateVariables.getSortedByLengthQueue();

         for (Pair<QuadEdge, QuadEdgeTriangle> edgeAndTriangle : queue)
         {
            QuadEdge edge = edgeAndTriangle.getLeft();
            Point3d dest = PolygonizerTools.toPointInWorld(edge.dest().getX(), edge.dest().getY(), planeOrigin, planeOrientation);
            Point3d orig = PolygonizerTools.toPointInWorld(edge.orig().getX(), edge.orig().getY(), planeOrigin, planeOrientation);
            boolean isEdgeTooLong = dest.distance(orig) > parameters.getEdgeLengthThreshold();
            Color lineColor = Color.hsb(regionColor.getHue(), regionColor.getSaturation(), isEdgeTooLong ? 0.25 : regionColor.getBrightness());
            meshBuilder.addLine(dest, orig, 0.0015, lineColor);

            QuadEdgeTriangle triangle = edgeAndTriangle.getRight();
            List<Point2d> triangleVerticesLocal = Arrays.stream(triangle.getVertices()).map(v -> new Point2d(v.getX(), v.getY())).collect(Collectors.toList());
            List<Point3d> triangleVerticesWorld = PolygonizerTools.toPointsInWorld(triangleVerticesLocal, planeOrigin, planeOrientation);
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

   private Node createConvexDecompositionGraphics(PlanarRegionSegmentationRawData rawData,
         ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      ConcaveHullCollection concaveHullCollection = concaveHullFactoryResult.getConcaveHullCollection();
      double depthThreshold = polygonizerParameters.getDepthThreshold();
      List<ConvexPolygon2d> convexPolygons = new ArrayList<>();
      ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHullCollection, depthThreshold, convexPolygons);

      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(64));

      int regionId = rawData.getRegionId();
      RigidBodyTransform rigidBodyTransform = rawData.getTransformFromLocalToWorld();
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

   private static Node createOrderedBorderEdgesGraphics(PlanarRegionSegmentationRawData rawData,
         ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(1024));
      Point3d planeOrigin = rawData.getOrigin();
      Quat4d planeOrientation = rawData.getOrientation();

      double startHue = 0.0;
      double endHue = 240.0;

      double lineStartBirghtness = 0.2;
      double lineEndBirghtness = 1.0;
      double minSaturation = 0.2;
      double maxSaturation = 0.9;
      double lineSat;
      double lineHue;

      List<ConcaveHullVariables> intermediateVariablesList = concaveHullFactoryResult.getIntermediateVariables();

      for (int variablesIndex = 0; variablesIndex < intermediateVariablesList.size(); variablesIndex++)
      {
         if (intermediateVariablesList.size() == 1)
         {
            lineSat = maxSaturation;
         }
         else
         {
            double alphaSat = variablesIndex / (double) (intermediateVariablesList.size() - 1.0);
            lineSat = (1.0 - alphaSat) * minSaturation + alphaSat * maxSaturation;
         }

         List<QuadEdge> orderedBorderEdges = intermediateVariablesList.get(variablesIndex).getOrderedBorderEdges();
         for (int edgeIndex = 0; edgeIndex < orderedBorderEdges.size(); edgeIndex++)
         {
            QuadEdge edge = orderedBorderEdges.get(edgeIndex);
            Point3d orig = PolygonizerTools.toPointInWorld(scaleX * edge.orig().getX(), scaleY * edge.orig().getY(), planeOrigin, planeOrientation);
            Point3d dest = PolygonizerTools.toPointInWorld(scaleX * edge.dest().getX(), scaleY * edge.dest().getY(), planeOrigin, planeOrientation);

            if (orderedBorderEdges.size() == 1)
            {
               lineHue = startHue;
            }
            else
            {
               double alphaHue = edgeIndex / (double) (orderedBorderEdges.size() - 1.0);
               lineHue = (1.0 - alphaHue) * startHue + alphaHue * endHue;
            }

            Color startColor = Color.hsb(lineHue, lineSat, lineStartBirghtness);
            Color endColor = Color.hsb(lineHue, lineSat, lineEndBirghtness);
            meshBuilder.addLine(orig, dest, 0.002, startColor, endColor);
         }
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
