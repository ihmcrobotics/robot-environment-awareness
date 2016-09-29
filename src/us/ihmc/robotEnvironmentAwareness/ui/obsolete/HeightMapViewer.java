package us.ihmc.robotEnvironmentAwareness.ui.obsolete;

import java.util.LinkedHashMap;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;

import javafx.application.Platform;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.ihmcPerception.depthData.DepthDataFilter;
import us.ihmc.robotics.hyperCubeTree.HyperCubeLeaf;
import us.ihmc.robotics.hyperCubeTree.HyperCubeTreeListener;
import us.ihmc.robotics.hyperCubeTree.OneDimensionalBounds;
import us.ihmc.robotics.quadTree.QuadTreeForGroundListener;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.GroundAirDescriptor;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.GroundOnlyQuadTreeData;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeForGroundHeightMap;
import us.ihmc.tools.thread.ThreadTools;

public class HeightMapViewer extends Group implements QuadTreeForGroundListener, HyperCubeTreeListener<GroundAirDescriptor, GroundOnlyQuadTreeData>
{
   private static final Color DEFAULT_COLOR = Color.DARKCYAN;
   private static final float INVALID_HEIGHT = Float.NEGATIVE_INFINITY;

   private ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("HeightMapViewer"));
   private ScheduledFuture<?> scheduled;

   private final LinkedHashMap<String, Box> boxes = new LinkedHashMap<>();
   private final ConcurrentLinkedDeque<String> idsOfBoxToRemove = new ConcurrentLinkedDeque<>();
   private final ConcurrentLinkedDeque<BoxParameters> parametersOfBoxToAdd = new ConcurrentLinkedDeque<>();
   private final ObservableList<Node> children;
   private QuadTreeForGroundHeightMap quadTree = DepthDataFilter.setupGroundOnlyQuadTree(DepthDataFilterParameters.getDefaultParameters());

   private final AtomicBoolean enable = new AtomicBoolean(true);
   private final AtomicBoolean clear = new AtomicBoolean(false);

   private double minZ = -0.5;

   public HeightMapViewer()
   {
      children = getChildren();
      quadTree.addQuadTreeListener(this);
      quadTree.setHeightThreshold(0.0);
   }

   @Override
   public void nodeAdded(String id, us.ihmc.robotics.quadTree.Box bounds, float x, float y, float height)
   {
      double lx = bounds.maxX - bounds.minX;
      double ly = bounds.maxY - bounds.minY;
      double lz = height - minZ;
      double z = height - lz / 2.0;

      BoxParameters boxParameters = new BoxParameters(id);
      boxParameters.setBoxLocation(x, y, z);
      boxParameters.setBoxSize(lx, ly, lz);
      parametersOfBoxToAdd.add(boxParameters);
   }

   @Override
   public void nodeAdded(String id, OneDimensionalBounds[] bounds, HyperCubeLeaf<GroundAirDescriptor> leaf)
   {
      float height = getHeightFromLeaf(leaf);
      if (height == INVALID_HEIGHT)
         return;

      double x = bounds[0].midpoint();
      double y = bounds[1].midpoint();
      double lx = bounds[0].size();
      double ly = bounds[1].size();
      double lz = height - minZ;
      double z = height - lz / 2.0;

      BoxParameters boxParameters = new BoxParameters(id);
      boxParameters.setBoxLocation(x, y, z);
      boxParameters.setBoxSize(lx, ly, lz);
      parametersOfBoxToAdd.add(boxParameters);
   }

   public float getHeightFromLeaf(final HyperCubeLeaf<GroundAirDescriptor> leaf)
   {
      if (leaf == null || leaf.getValue() == null || leaf.getValue().getHeight() == null)
         return INVALID_HEIGHT;

      return leaf.getValue().getHeight();
   }

   @Override
   public void nodeRemoved(String id)
   {
      idsOfBoxToRemove.add(id);
   }

   private Runnable createUpdater()
   {
      Runnable runnable = new Runnable()
      {
         @Override
         public void run()
         {
            if (clear.getAndSet(false))
            {
               quadTree.clear();
               return;
            }

            while (!idsOfBoxToRemove.isEmpty())
            {
               final Box boxToRemove = boxes.remove(idsOfBoxToRemove.poll());
               if (boxToRemove != null && boxToRemove.getParent() != null)
               {
                  Platform.runLater(new Runnable()
                  {
                     @Override
                     public void run()
                     {
                        children.remove(boxToRemove);
                     }
                  });
               }
            }

            if (!enable.get())
               return;

            for (int i = 0; i < 10; i++)
            {
               if (parametersOfBoxToAdd.isEmpty())
                  break;
               final BoxParameters poll = parametersOfBoxToAdd.poll();
               Platform.runLater(new Runnable()
               {
                  @Override
                  public void run()
                  {
                     updateBox(poll);
                  }
               });
            }

         }
      };

      return runnable;
   }

   private void updateBox(BoxParameters boxParameters)
   {
      String boxId = boxParameters.getBoxId();
      Box box = boxes.get(boxId);
      if (box == null)
      {
         box = new Box();
         boxes.put(boxId, box);
         PhongMaterial material = new PhongMaterial();
         material.setDiffuseColor(DEFAULT_COLOR);
         material.setSpecularColor(DEFAULT_COLOR.brighter());
         box.setMaterial(material);
         children.add(box);
      }

      boxParameters.updateBox(box);
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
      PacketConsumer<PointCloudWorldPacket> packetConsumer = new PacketConsumer<PointCloudWorldPacket>()
      {
         @Override
         public void receivedPacket(PointCloudWorldPacket packet)
         {
            if (packet == null || !enable.get())
               return;

            Point3f[] data = packet.getDecayingWorldScan();
            int packetSize = data.length;
            int size = packetSize;
            size = Math.min(size, 100);

            for (int i = 0; i < size; i++)
            {
               int index = i * packetSize / size;
               Point3f dataPoint = data[index];
//               Point3f dataPoint = data[i];
               double x = dataPoint.x;
               double y = dataPoint.y;
               double z = dataPoint.z;
               quadTree.addToQuadtree(x, y, z);
            }
         }
      };
      return packetConsumer;
   }

   private static class BoxParameters
   {
      private final String id;
      private final Point3d location = new Point3d();
      private final Vector3d size = new Vector3d();

      public BoxParameters(String id)
      {
         this.id = id;
      }

      private void setBoxLocation(double x, double y, double z)
      {
         location.set(x, y, z);
      }

      private void setBoxSize(double lx, double ly, double lz)
      {
         size.set(lx, ly, lz);
      }

      private void updateBox(Box boxToUpdate)
      {
         boxToUpdate.setWidth(size.getX());
         boxToUpdate.setHeight(size.getY());
         boxToUpdate.setDepth(size.getZ());
         boxToUpdate.setTranslateX(location.getX());
         boxToUpdate.setTranslateY(location.getY());
         boxToUpdate.setTranslateZ(location.getZ());
      }

      public String getBoxId()
      {
         return id;
      }
   }

   @Override
   public void leafAdded(HyperCubeLeaf<GroundAirDescriptor> leaf)
   {
   }

   @Override
   public void metaDataUpdated(String id, OneDimensionalBounds[] bounds, GroundOnlyQuadTreeData data)
   {
      if (!data.isHeightValid())
         return;
      double height = data.getHeight();
      double x = bounds[0].midpoint();
      double y = bounds[1].midpoint();
      double lx = bounds[0].size();
      double ly = bounds[1].size();
      double lz = height - minZ;
      double z = height - lz / 2.0;

      BoxParameters boxParameters = new BoxParameters(id);
      boxParameters.setBoxLocation(x, y, z);
      boxParameters.setBoxSize(lx, ly, lz);
      parametersOfBoxToAdd.add(boxParameters);
   }

   @Override
   public void treeCleared()
   {
      Platform.runLater(new Runnable()
      {
         @Override
         public void run()
         {
            boxes.clear();
            children.clear();
         }
      });
   }

   private BooleanProperty clearProperty;

   public BooleanProperty clearHeightMapProperty()
   {
      if (clearProperty == null)
      {
         clearProperty = new SimpleBooleanProperty(this, "clearHeightMapViewer", false)
         {
            @Override
            protected void invalidated()
            {
               if (get())
               {
                  clear.set(true);
                  set(false);
               }
            }
         };
      }
      return clearProperty;
   }

   private BooleanProperty enableProperty;

   public BooleanProperty enableProperty()
   {
      if (enableProperty == null)
      {
         enableProperty = new SimpleBooleanProperty(this, "enableHeightMapViwer", enable.get())
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

   private HeightMapPropertyControlFactory uiControlFactory;

   public HeightMapPropertyControlFactory uiControlFactory()
   {
      if (uiControlFactory == null)
         uiControlFactory = new HeightMapPropertyControlFactory(this);
      return uiControlFactory;
   }

   @Override
   public void RawPointAdded(float x, float y, float z)
   {
   }

   @Override
   public void PopToOctree(Point3f location)
   {
   }

   @Override
   public void PopToOctree(Point3f location, Point3f LidarHeadLocation)
   {
   }
}
