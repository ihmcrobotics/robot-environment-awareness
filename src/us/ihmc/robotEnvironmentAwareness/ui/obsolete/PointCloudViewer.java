package us.ihmc.robotEnvironmentAwareness.ui.obsolete;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

import javafx.application.Platform;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Sphere;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.tools.thread.ThreadTools;

public class PointCloudViewer extends Group
{
   private static final Color DEFAULT_COLOR = Color.BLUE;

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

   public PointCloudViewer()
   {
      children = getChildren();
   }

   public Runnable createUpdater()
   {
      Runnable runnable = new Runnable()
      {
         @Override
         public void run()
         {
            try
            {
               if (clearPointCloud.getAndSet(false))
               {
                  clear();
                  return;
               }

               if (!enable.get())
                  return;

               final List<Point3d> pointsToUpdate = new ArrayList<>();
               
               for (int i = 0; i < 10; i++)
               {
                  Point3d poll = newPointCoordinates.poll();
                  if (poll != null)
                     pointsToUpdate.add(poll);
                  else
                     break;
               }
               
               Platform.runLater(new Runnable()
               {
                  @Override
                  public void run()
                  {
                     addPoints(pointsToUpdate);
                  }
               });
            }
            catch(Exception e)
            {
               e.printStackTrace();
            }
         }
      };
      return runnable;
   }

   private void clear()
   {
      Platform.runLater(new Runnable()
      {
         @Override
         public void run()
         {
            children.clear();
            index.set(0);
         }
      });
   }

   public void addPoints(Point3d[] pointLocations)
   {
      for (Point3d pointLocation : pointLocations)
         addPoint(pointLocation);
   }

   public void addPoints(Iterable<Point3d> pointLocations)
   {
      for (Point3d pointLocation : pointLocations)
         addPoint(pointLocation);
   }

   public void addPoint(Point3d pointLocation)
   {
      addPoint(pointLocation, DEFAULT_COLOR);
   }

   public void addPoint(Point3d pointLocation, Color pointColor)
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

   private SimpleIntegerProperty sizeProperty;

   public IntegerProperty pointCloudSizeProperty()
   {
      if (sizeProperty == null)
      {
         sizeProperty = new SimpleIntegerProperty(this, "pointCloudSize", maximumSize.get())
         {
            @Override
            protected void invalidated()
            {
               maximumSize.set(intValue());
            }
         };
      }
      return sizeProperty;
   }

   private BooleanProperty clearProperty;

   public BooleanProperty clearPointCloudProperty()
   {
      if (clearProperty == null)
      {
         clearProperty = new SimpleBooleanProperty(this, "clearPointCloud", false)
         {
            @Override
            protected void invalidated()
            {
               if (get())
               {
                  clearPointCloud.set(true);
                  set(false);
               }
            }
         };
      }
      return clearProperty;
   }

   private BooleanProperty enableProperty;

   public BooleanProperty enableViewerProperty()
   {
      if (enableProperty == null)
      {
         enableProperty = new SimpleBooleanProperty(this, "enablePointCloudViewer", false)
         {
            @Override
            protected void invalidated()
            {
               enable.set(get());
            }
         };
      }
      return enableProperty;
   }

   public PacketConsumer<PointCloudWorldPacket> getPointCloudWorldPacketConsumer()
   {
      PacketConsumer<PointCloudWorldPacket> packetConsumer = new PacketConsumer<PointCloudWorldPacket>()
      {
         @Override
         public void receivedPacket(PointCloudWorldPacket packet)
         {
            if (packet == null || !enable.get())
               return;

            Point3f[] decayingWorldScan = packet.getDecayingWorldScan();
            int packetSize = decayingWorldScan.length;
            int size = packetSize;
            size = Math.min(size, 1000);

            for (int i = 0; i < size; i++)
            {
               if (newPointCoordinates.size() == maximumDequeSize)
                  break;

               int index = i * packetSize / size;
               Point3d pointToAdd = new Point3d(decayingWorldScan[index]);
               newPointCoordinates.add(pointToAdd);
            }
         }
      };
      return packetConsumer;
   }

   private PointCloudPropertyControlFactory uiControlFactory;

   public PointCloudPropertyControlFactory uiControlFactory()
   {
      if (uiControlFactory == null)
      {
         uiControlFactory = new PointCloudPropertyControlFactory(this);
      }
      return uiControlFactory;
   }
}
