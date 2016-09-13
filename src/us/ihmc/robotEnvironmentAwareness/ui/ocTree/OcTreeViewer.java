package us.ihmc.robotEnvironmentAwareness.ui.ocTree;

import java.util.concurrent.atomic.AtomicBoolean;

import javafx.animation.AnimationTimer;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.Group;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.octoMap.boundingBox.OcTreeSimpleBoundingBox;
import us.ihmc.octoMap.occupancy.OccupancyParameters;
import us.ihmc.robotEnvironmentAwareness.ui.ocTree.OcTreeGraphicsBuilder.ColoringType;

public class OcTreeViewer extends Group
{
   private final AtomicBoolean enable;
   private final AtomicBoolean clear = new AtomicBoolean(false);

   private final MeshView occupiedLeafsMeshView = new MeshView();
   private final MeshView freeLeafsMeshView = new MeshView();

   private final OcTreeUIController controller;
   private final AnimationTimer renderMeshAnimation;

   public OcTreeViewer(OcTreeUIController controller, boolean enableInitialValue)
   {
      this.controller = controller;
      enable = new AtomicBoolean(enableInitialValue);
      getChildren().addAll(occupiedLeafsMeshView, freeLeafsMeshView);
      setMouseTransparent(true);

      renderMeshAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            if (Thread.interrupted())
               return;

            if (clear.getAndSet(false))
            {
               occupiedLeafsMeshView.setMesh(null);
               freeLeafsMeshView.setMesh(null);
               return;
            }

            if (!controller.hasProcessedClear()) // Wait until the update thread has cleared.
               return;

            if (controller.hasNewOccupiedMeshToRender())
            {
               updateMeshView(occupiedLeafsMeshView, controller.pollOccupiedMesh());
            }

            if (controller.hasNewFreeMeshToRender())
            {
               updateMeshView(freeLeafsMeshView, controller.pollFreeMesh());
            }
         }
      };
   }

   public void start()
   {
      renderMeshAnimation.start();
      controller.start();
   }

   public void stop()
   {
      renderMeshAnimation.stop();
      controller.stop();
   }

   private void updateMeshView(MeshView meshViewToUpdate, Pair<Mesh, Material> meshMaterial)
   {
      meshViewToUpdate.setMesh(meshMaterial.getKey());
      meshViewToUpdate.setMaterial(meshMaterial.getValue());
   }

   private BooleanProperty clearProperty;

   public BooleanProperty clearOcTreeProperty()
   {
      if (clearProperty == null)
      {
         clearProperty = new SimpleBooleanProperty(this, "clear" + getClass().getSimpleName(), false)
         {
            @Override
            protected void invalidated()
            {
               if (get())
               {
                  clear.set(true);
                  controller.clear();
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
         enableProperty = new SimpleBooleanProperty(this, "enable" + getClass().getSimpleName(), enable.get())
         {
            @Override
            protected void invalidated()
            {
               enable.set(get());
               controller.setEnable(get());
            }
         };
      }
      return enableProperty;
   }

   private IntegerProperty viewerDepthProperty;

   public IntegerProperty viewerDepthProperty()
   {
      if (viewerDepthProperty == null)
      {
         viewerDepthProperty = new SimpleIntegerProperty(this, "viewerDepth", controller.getCurrentTreeDepthForDisplay())
         {
            @Override
            protected void invalidated()
            {
               controller.setTreeDepthForDisplay(get());
            }
         };
      }
      return viewerDepthProperty;
   }

   public int getMaximumTreeDepth()
   {
      return controller.getMaximumTreeDepth();
   }

   private BooleanProperty showFreeSpaceProperty;

   public BooleanProperty showFreeSpaceProperty()
   {
      if (showFreeSpaceProperty == null)
      {
         showFreeSpaceProperty = new SimpleBooleanProperty(this, "showFreeSpace", controller.isShowingFreeSpace())
         {
            @Override
            protected void invalidated()
            {
               controller.showFreeSpace(get());
            }
         };
      }
      return showFreeSpaceProperty;
   }

   private ObjectProperty<ColoringType> coloringTypeProperty;

   public ObjectProperty<ColoringType> coloringTypeProperty()
   {
      if (coloringTypeProperty == null)
      {
         coloringTypeProperty = new SimpleObjectProperty<OcTreeGraphicsBuilder.ColoringType>(this, "OcTreeColoringType", controller.getColoringType())
         {
            @Override
            protected void invalidated()
            {
               controller.setColoringType(get());
            }
         };
      }
      return coloringTypeProperty;
   }

   private BooleanProperty showEstimatedSurfacesProperty;

   public BooleanProperty showEstimatedSurfacesProperty()
   {
      if (showEstimatedSurfacesProperty == null)
      {
         showEstimatedSurfacesProperty = new SimpleBooleanProperty(this, "showEstimatedSurfaces", controller.isShowingEstimatedSurfaces())
         {
            @Override
            protected void invalidated()
            {
               controller.showEstimatedSurfaces(get());
            }
         };
      }
      return showEstimatedSurfacesProperty;
   }

   private DoubleProperty occupancyThresholdProperty;

   public DoubleProperty occupancyThresholdProperty()
   {
      if (occupancyThresholdProperty == null)
      {
         occupancyThresholdProperty = new SimpleDoubleProperty(this, "ocTreeOccupancyThresholdProperty", controller.getOcTreeOccupancyParameters().getOccupancyThreshold())
         {
            @Override
            protected void invalidated()
            {
               OccupancyParameters newParameters = new OccupancyParameters(controller.getOcTreeOccupancyParameters());
               newParameters.setOccupancyThreshold(get());
               controller.setOcTreeOccupancyParameters(newParameters);
            }
         };
      }
      return occupancyThresholdProperty;
   }

   private DoubleProperty hitUpdateProperty;

   public DoubleProperty hitUpdateProperty()
   {
      if (hitUpdateProperty == null)
      {
         hitUpdateProperty = new SimpleDoubleProperty(this, "ocTreeHitUpdateProperty", controller.getOcTreeOccupancyParameters().getHitProbability())
         {
            @Override
            protected void invalidated()
            {
               OccupancyParameters newParameters = new OccupancyParameters(controller.getOcTreeOccupancyParameters());
               newParameters.setHitProbabilityUpdate(get());
               controller.setOcTreeOccupancyParameters(newParameters);
            }
         };
      }
      return hitUpdateProperty;
   }

   private DoubleProperty missUpdateProperty;

   public DoubleProperty missUpdateProperty()
   {
      if (missUpdateProperty == null)
      {
         missUpdateProperty = new SimpleDoubleProperty(this, "ocTreeMissUpdateProperty", controller.getOcTreeOccupancyParameters().getMissProbability())
         {
            @Override
            protected void invalidated()
            {
               OccupancyParameters newParameters = new OccupancyParameters(controller.getOcTreeOccupancyParameters());
               newParameters.setMissProbabilityUpdate(get());
               controller.setOcTreeOccupancyParameters(newParameters);
            }
         };
      }
      return missUpdateProperty;
   }

   private DoubleProperty minProbabilityProperty;

   public DoubleProperty minProbabilityProperty()
   {
      if (minProbabilityProperty == null)
      {
         minProbabilityProperty = new SimpleDoubleProperty(this, "ocTreeMinimumProbabilityProperty", controller.getOcTreeOccupancyParameters().getMinProbability())
         {
            @Override
            protected void invalidated()
            {
               OccupancyParameters newParameters = new OccupancyParameters(controller.getOcTreeOccupancyParameters());
               newParameters.setMinProbability(get());
               controller.setOcTreeOccupancyParameters(newParameters);
            }
         };
      }
      return minProbabilityProperty;
   }

   private DoubleProperty maxProbabilityProperty;

   public DoubleProperty maxProbabilityProperty()
   {
      if (maxProbabilityProperty == null)
      {
         maxProbabilityProperty = new SimpleDoubleProperty(this, "ocTreeMaximumProbabilityProperty", controller.getOcTreeOccupancyParameters().getMaxProbability())
         {
            @Override
            protected void invalidated()
            {
               OccupancyParameters newParameters = new OccupancyParameters(controller.getOcTreeOccupancyParameters());
               newParameters.setMaxProbability(get());
               controller.setOcTreeOccupancyParameters(newParameters);
            }
         };
      }
      return maxProbabilityProperty;
   }

   private BooleanProperty enableBoundingBoxProperty;

   public BooleanProperty enableBoundingBoxProperty()
   {
      if (enableBoundingBoxProperty == null)
      {
         enableBoundingBoxProperty = new SimpleBooleanProperty(this, "enableBoundingBoxProperty", controller.isBoundingBoxEnabled())
         {
            @Override
            protected void invalidated()
            {
               controller.enableBoundingBox(get());
            }
         };
      }
      return enableBoundingBoxProperty;
   }

   private ObjectProperty<OcTreeSimpleBoundingBox> boundingBoxProperty;

   public ObjectProperty<OcTreeSimpleBoundingBox> boundingBoxProperty()
   {
      if (boundingBoxProperty == null)
      {
         boundingBoxProperty = new SimpleObjectProperty<OcTreeSimpleBoundingBox>(this, "ocTreeBoundingBoxProperty", controller.getBoundingBox())
         {
            @Override
            protected void invalidated()
            {
               if (get() != null)
                  controller.setBoundingBox(get());
            }
         };
      }
      return boundingBoxProperty;
   }

   private DoubleProperty ocTreeMinRangeProperty;

   public DoubleProperty ocTreeMinRangeProperty()
   {
      if (ocTreeMinRangeProperty == null)
      {
         ocTreeMinRangeProperty = new SimpleDoubleProperty(this, "ocTreeMinRangeProperty", controller.getMinRange())
         {
            @Override
            protected void invalidated()
            {
               if (!Double.isNaN(get()))
                  controller.setMinRange(get());
            }
         };
      }
      return ocTreeMinRangeProperty;
   }

   private DoubleProperty ocTreeMaxRangeProperty;

   public DoubleProperty ocTreeMaxRangeProperty()
   {
      if (ocTreeMaxRangeProperty == null)
      {
         ocTreeMaxRangeProperty = new SimpleDoubleProperty(this, "ocTreeMaxRangeProperty", controller.getMaxRange())
         {
            @Override
            protected void invalidated()
            {
               if (!Double.isNaN(get()))
                  controller.setMaxRange(get());
            }
         };
      }
      return ocTreeMaxRangeProperty;
   }

   private OcTreeUIControlFactory uiControlFactory;

   public OcTreeUIControlFactory uiControlFactory()
   {
      if (uiControlFactory == null)
         uiControlFactory = new OcTreeUIControlFactory(this);
      return uiControlFactory;
   }
}
