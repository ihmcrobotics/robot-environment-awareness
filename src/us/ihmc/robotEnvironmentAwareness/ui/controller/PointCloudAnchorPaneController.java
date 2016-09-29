package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import javax.vecmath.Point3d;

import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.ToggleButton;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.tools.thread.ThreadTools;

public class PointCloudAnchorPaneController
{
   private static final int MAX_NUMBER_POINTS_TO_PROCESS = 1000;

   private static final Color DEFAULT_COLOR = Color.BLUE;

   @FXML
   private ToggleButton enableButton;
   @FXML
   private Button clearButton;

   private final Group pointCloudRoot = new Group();

   private ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("PointCloudViewer"));
   private ScheduledFuture<?> scheduled;

   private final int maximumDequeSize = 5000;
   private final ConcurrentLinkedDeque<Point3d> newPointCoordinates = new ConcurrentLinkedDeque<>();

   private double pointSize = 0.01;
   private final AtomicInteger index = new AtomicInteger(0);
   private final AtomicInteger maximumSize = new AtomicInteger(20000);
   private final AtomicBoolean clearPointCloud = new AtomicBoolean(false);
   private final AtomicBoolean enable = new AtomicBoolean(false);
   private final ObservableList<Node> children;

   public PointCloudAnchorPaneController()
   {
      children = pointCloudRoot.getChildren();
   }

   public void bindControls()
   {
      enableButton.selectedProperty().addListener((ChangeListener<Boolean>) (observable, oldValue, newValue) -> enable.set(newValue));
      Platform.runLater(() -> enableButton.setSelected(enable.get()));
   }

   private Runnable createUpdater()
   {
      return this::processUpdate;
   }

   public void processUpdate()
   {
      try
      {
         if (clearPointCloud.getAndSet(false))
         {
            Platform.runLater(this::clearNow);
            return;
         }

         if (!enable.get())
            return;

         for (int i = 0; i < 10; i++)
         {
            Point3d poll = newPointCoordinates.poll();
            if (poll == null)
               break;

            Platform.runLater(() -> addPoint(poll));
         }

      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   @FXML
   public void clear()
   {
      clearPointCloud.set(true);
   }

   private void clearNow()
   {
      children.clear();
      index.set(0);
   }

   private void addPoint(Point3d pointLocation)
   {
      addPoint(pointLocation, DEFAULT_COLOR);
   }

   private void addPoint(Point3d pointLocation, Color pointColor)
   {
      Node point;

      if (children.size() <= index.get())
      {
         Sphere sphere = new Sphere(pointSize, 1);
         PhongMaterial material = new PhongMaterial();
         material.setDiffuseColor(pointColor);
         material.setSpecularColor(pointColor.brighter());
         sphere.setMaterial(material);
         children.add(sphere);
         point = sphere;
      }
      else
      {
         point = children.get(index.get());
      }

      point.setTranslateX(pointLocation.getX());
      point.setTranslateY(pointLocation.getY());
      point.setTranslateZ(pointLocation.getZ());

      index.incrementAndGet();

      if (index.get() == maximumSize.get() - 1)
         index.set(0);

      while (children.size() > maximumSize.get())
         children.remove(children.size() - 1);
   }

   public void start()
   {
      if (scheduled == null)
         scheduled = executorService.scheduleAtFixedRate(createUpdater(), 0, 10, TimeUnit.MILLISECONDS);
   }

   public void stop()
   {
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

   public PacketConsumer<PointCloudWorldPacket> getPointCloudWorldPacketConsumer()
   {
      return (packet) -> processNewPacket(packet);
   }

   private void processNewPacket(PointCloudWorldPacket packet)
   {
      if (packet == null || !enable.get())
         return;

      float[] scan = packet.decayingWorldScan;
      int numberOfPoints = scan.length / 3;
      int numberOfPointsToRead = Math.min(numberOfPoints, MAX_NUMBER_POINTS_TO_PROCESS);
      int increment = numberOfPoints / numberOfPointsToRead;

      for (int i = 0; i < numberOfPoints; i += increment)
      {
         if (newPointCoordinates.size() == maximumDequeSize)
            break;

         int xIndex = i * 3;
         int yIndex = xIndex + 1;
         int zIndex = yIndex + 1;
         newPointCoordinates.add(new Point3d(scan[xIndex], scan[yIndex], scan[zIndex]));
      }
   }

   public Node getRoot()
   {
      return pointCloudRoot;
   }
}
