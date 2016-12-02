package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3f;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.beans.Observable;
import javafx.beans.value.ChangeListener;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.graphics3DDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.tools.thread.ThreadTools;

public class PointCloudAnchorPaneController extends REABasicUIController
{
   private static final float SCAN_POINT_SIZE = 0.01f;

   private static final Material defaultMaterial = new PhongMaterial(Color.DARKRED);

   @FXML
   private ToggleButton enableButton;
   @FXML
   private Slider scanHistorySizeSlider;

   private final Group root = new Group();

   private ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("PointCloudViewer"));
   private ScheduledFuture<?> scheduled;

   private final AtomicBoolean clear = new AtomicBoolean(false);
   private final AtomicBoolean enable = new AtomicBoolean(false);
   private final AtomicInteger numberOfScans = new AtomicInteger(20);
   private final AtomicInteger currentScanIndex = new AtomicInteger(0);
   private AtomicReference<LidarScanMessage> newMessageToRender;
   private final AtomicReference<MeshView> scanMeshToRender = new AtomicReference<>(null);
   private final JavaFXMultiColorMeshBuilder scanMeshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(128));
   private final ObservableList<Node> children;

   private final AnimationTimer pointCloudAnimation;

   public PointCloudAnchorPaneController()
   {
      children = root.getChildren();
      pointCloudAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            render();
         }
      };
   }

   public void bindControls()
   {
      newMessageToRender = uiMessager.createInput(REAModuleAPI.LidarScanState);

      enableButton.selectedProperty().addListener((ChangeListener<Boolean>) (observable, oldValue, newValue) -> enable.set(newValue));
      Platform.runLater(() -> enableButton.setSelected(enable.get()));
      scanHistorySizeSlider.valueProperty().addListener(this::updateNumberOfScans);
   }

   private void updateNumberOfScans(Observable observable)
   {
      numberOfScans.set((int) scanHistorySizeSlider.getValue());
   }

   private void render()
   {
      MeshView newScanMeshView = scanMeshToRender.getAndSet(null);

      if (clear.getAndSet(false))
      {
         children.clear();
         currentScanIndex.set(0);
      }

      while (children.size() > numberOfScans.get())
         children.remove(children.size() - 1);

      if (!enable.get())
         return;

      if (newScanMeshView != null)
      {
         if (children.size() <= currentScanIndex.get())
            children.add(newScanMeshView);
         else
            children.set(currentScanIndex.get(), newScanMeshView);

         for (int i = currentScanIndex.get() + 1; i < currentScanIndex.get() + children.size(); i++)
            ((MeshView) children.get(i % children.size())).setMaterial(defaultMaterial);

         currentScanIndex.set((currentScanIndex.get() + 1) % numberOfScans.get());
      }
   }

   private void computeScanMesh()
   {
      LidarScanMessage message = newMessageToRender.getAndSet(null);

      uiMessager.submitStateRequestToModule(REAModuleAPI.RequestLidarScan);

      if (message == null)
         return;

      Point3f scanPoint = new Point3f();
      scanMeshBuilder.clear();
      for (int i = 0; i < message.getNumberOfScanPoints(); i++)
      {
         double alpha = i / (double) message.getNumberOfScanPoints();
         Color color = Color.hsb(alpha * 240.0, 1.0, 1.0);

         message.getScanPoint(i, scanPoint);

         scanMeshBuilder.addMesh(MeshDataGenerator.Tetrahedron(SCAN_POINT_SIZE), scanPoint, color);
      }

      MeshView scanMeshView = new MeshView(scanMeshBuilder.generateMesh());
      scanMeshView.setMaterial(scanMeshBuilder.generateMaterial());
      scanMeshToRender.set(scanMeshView);
      scanMeshBuilder.clear();
   }

   @FXML
   public void clear()
   {
      clear.set(true);
   }

   public void start()
   {
      pointCloudAnimation.start();

      if (scheduled == null)
         scheduled = executorService.scheduleAtFixedRate(this::computeScanMesh, 0, 10, TimeUnit.MILLISECONDS);
   }

   public void stop()
   {
      pointCloudAnimation.stop();

      if (scheduled != null)
      {
         scheduled.cancel(true);
         scheduled = null;
      }

      if (executorService != null)
      {
         executorService.shutdown();
         executorService = null;
      }
   }

   public Node getRoot()
   {
      return root;
   }
}
