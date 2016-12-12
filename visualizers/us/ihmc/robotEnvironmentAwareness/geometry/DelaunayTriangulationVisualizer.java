package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import com.vividsolutions.jts.geom.MultiPoint;
import com.vividsolutions.jts.triangulate.ConformingDelaunayTriangulationBuilder;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdge;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdgeSubdivision;

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
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataImporter;

public class DelaunayTriangulationVisualizer extends Application
{
   private static final boolean VISUALIZE_EDGES = false;
   private static final boolean VISUALIZE_PRIMARY_EDGES = true;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle(getClass().getSimpleName());

      PlanarRegionSegmentationDataImporter dataImporter = PlanarRegionSegmentationDataImporter.createImporterWithFileChooser(primaryStage);
      if (dataImporter == null)
         Platform.exit();
      dataImporter.loadPlanarRegionSegmentationData();
      List<PlanarRegionSegmentationMessage> planarRegionSegmentationData = dataImporter.getPlanarRegionSegmentationData();

      View3DFactory view3dFactory = new View3DFactory(600, 400);
      view3dFactory.addCameraController();
      view3dFactory.addWorldCoordinateSystem(0.3);

      Map<Node, Integer> nodeToRegionId = new HashMap<>();

      Point3d average = PolygonizerVisualizer.computeAverage(planarRegionSegmentationData);
      average.negate();

      for (PlanarRegionSegmentationMessage planarRegionSegmentationMessage : planarRegionSegmentationData)
      {
         Node regionGraphics = createRegionGraphics(planarRegionSegmentationMessage);
         regionGraphics.setManaged(false);
         PolygonizerVisualizer.translateNode(regionGraphics, average);
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

   private static Node createRegionGraphics(PlanarRegionSegmentationMessage planarRegionSegmentationMessage)
   {
      Group regionGroup = new Group();
      ObservableList<Node> children = regionGroup.getChildren();

      QuadEdgeSubdivision quadEdgeSubdivision = createQuadEdgeSubdivision(planarRegionSegmentationMessage);
      if (VISUALIZE_EDGES)
         children.add(createEdgesGraphics(quadEdgeSubdivision, planarRegionSegmentationMessage));
      if (VISUALIZE_PRIMARY_EDGES)
         children.add(createPrimaryEdgesGraphics(quadEdgeSubdivision, planarRegionSegmentationMessage));
      return regionGroup;
   }

   private static QuadEdgeSubdivision createQuadEdgeSubdivision(PlanarRegionSegmentationMessage planarRegionSegmentationMessage)
   {
      List<Point2d> point2ds = PolygonizerTools.extractPointsInPlane(planarRegionSegmentationMessage);
      MultiPoint multiPoint = SimpleConcaveHullFactory.createMultiPoint(point2ds);

      ConformingDelaunayTriangulationBuilder conformingDelaunayTriangulationBuilder = new ConformingDelaunayTriangulationBuilder();
      conformingDelaunayTriangulationBuilder.setSites(multiPoint);
      return conformingDelaunayTriangulationBuilder.getSubdivision();
   }

   @SuppressWarnings("unchecked")
   private static Node createEdgesGraphics(QuadEdgeSubdivision quadEdgeSubdivision, PlanarRegionSegmentationMessage planarRegionSegmentationMessage)
   {
      List<QuadEdge> edges = (List<QuadEdge>) quadEdgeSubdivision.getEdges();

      int regionId = planarRegionSegmentationMessage.getRegionId();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(16));
      Point3d planeOrigin = new Point3d(planarRegionSegmentationMessage.getOrigin());
      Quat4d planeOrientation = PolygonizerTools.getRotationBasedOnNormal(planarRegionSegmentationMessage.getNormal());
      Color regionColor = OcTreeMeshBuilder.getRegionColor(regionId);

      for (QuadEdge edge : edges)
      {
         Point3d dest = PolygonizerTools.toPointInWorld(edge.dest().getX(), edge.dest().getY(), planeOrigin, planeOrientation);
         Point3d orig = PolygonizerTools.toPointInWorld(edge.orig().getX(), edge.orig().getY(), planeOrigin, planeOrientation);
         meshBuilder.addLine(dest, orig, 0.0015, regionColor);
      }
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      meshView.setMouseTransparent(true);
      return meshView;
   }

   @SuppressWarnings("unchecked")
   private static Node createPrimaryEdgesGraphics(QuadEdgeSubdivision quadEdgeSubdivision, PlanarRegionSegmentationMessage planarRegionSegmentationMessage)
   {
      List<QuadEdge> primaryEdges = (List<QuadEdge>) quadEdgeSubdivision.getPrimaryEdges(false);

      int regionId = planarRegionSegmentationMessage.getRegionId();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(16));
      Point3d planeOrigin = new Point3d(planarRegionSegmentationMessage.getOrigin());
      Quat4d planeOrientation = PolygonizerTools.getRotationBasedOnNormal(planarRegionSegmentationMessage.getNormal());
      Color regionColor = OcTreeMeshBuilder.getRegionColor(regionId);

      for (QuadEdge edge : primaryEdges)
      {
         Point3d dest = PolygonizerTools.toPointInWorld(edge.dest().getX(), edge.dest().getY(), planeOrigin, planeOrientation);
         Point3d orig = PolygonizerTools.toPointInWorld(edge.orig().getX(), edge.orig().getY(), planeOrigin, planeOrientation);
         meshBuilder.addLine(dest, orig, 0.0015, regionColor);
      }
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      meshView.setMouseTransparent(true);
      return meshView;
   }

   @SuppressWarnings("unchecked")
   private static Node createOrderedBorderEdgesGraphics(QuadEdgeSubdivision quadEdgeSubdivision, PlanarRegionSegmentationMessage planarRegionSegmentationMessage)
   {
      List<QuadEdge> primaryEdges = (List<QuadEdge>) quadEdgeSubdivision.getPrimaryEdges(false);

      int regionId = planarRegionSegmentationMessage.getRegionId();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(16));
      Point3d planeOrigin = new Point3d(planarRegionSegmentationMessage.getOrigin());
      Quat4d planeOrientation = PolygonizerTools.getRotationBasedOnNormal(planarRegionSegmentationMessage.getNormal());
      Color regionColor = OcTreeMeshBuilder.getRegionColor(regionId);

      for (QuadEdge edge : primaryEdges)
      {
         Point3d dest = PolygonizerTools.toPointInWorld(edge.dest().getX(), edge.dest().getY(), planeOrigin, planeOrientation);
         Point3d orig = PolygonizerTools.toPointInWorld(edge.orig().getX(), edge.orig().getY(), planeOrigin, planeOrientation);
         meshBuilder.addLine(dest, orig, 0.0015, regionColor);
      }
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      meshView.setMouseTransparent(true);
      return meshView;
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
