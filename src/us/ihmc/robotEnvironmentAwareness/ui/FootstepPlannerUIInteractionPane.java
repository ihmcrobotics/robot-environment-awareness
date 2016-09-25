package us.ihmc.robotEnvironmentAwareness.ui;

import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.GridPane;
import us.ihmc.robotEnvironmentAwareness.ui.ocTree.OcTreeGraphicsBuilder.ColoringType;
import us.ihmc.robotEnvironmentAwareness.ui.ocTree.OcTreeUIControlFactory;

public class FootstepPlannerUIInteractionPane extends GridPane
{
   private PointCloudPropertyControlFactory pointCloudUIControlFactory;
   private HeightMapPropertyControlFactory heightMapUIControlFactory;
   private OcTreeUIControlFactory ocTreeUIControlFactory;

   public FootstepPlannerUIInteractionPane(PointCloudPropertyControlFactory pointCloudUIControlFactory, HeightMapPropertyControlFactory heightMapUIControlFactory,
         OcTreeUIControlFactory ocTreeUIControlFactory)
   {
      this.pointCloudUIControlFactory = pointCloudUIControlFactory;
      this.heightMapUIControlFactory = heightMapUIControlFactory;
      this.ocTreeUIControlFactory = ocTreeUIControlFactory;
      int row = 0;
      
      Button clearPointCloudButton = pointCloudUIControlFactory.clearPointCloudButton();
      ToggleButton enablePointCloudViewerButton = pointCloudUIControlFactory.enablePointCloudViewerButton();
      Slider pointCloudSizeSlider = pointCloudUIControlFactory.pointCloudSizeSlider();
      Label pointCloudSizeLabel = new Label("Point cloud size: ");
      pointCloudSizeLabel.setLabelFor(pointCloudSizeSlider);
      add(enablePointCloudViewerButton, 0, row);
      add(clearPointCloudButton, 1, row);
      add(pointCloudSizeLabel, 3, row);
      add(pointCloudSizeSlider, 4, row);

      row++;

      if (heightMapUIControlFactory != null)
      {
         Button clearHeightMapButton = heightMapUIControlFactory.clearHeightMapButton();
         ToggleButton enableHeightMapViewerButton = heightMapUIControlFactory.enableHeightMapViewerButton();
         add(enableHeightMapViewerButton, 0, row);
         add(clearHeightMapButton, 1, row);
         row++;
      }

      if (ocTreeUIControlFactory != null)
      {
         Button clearOcTreeButton = ocTreeUIControlFactory.clearOcTreeButton();
         ToggleButton enableOcTreeViewerButton = ocTreeUIControlFactory.enableOcTreeViewerButton();
         ToggleButton showFreeSpaceButton = ocTreeUIControlFactory.showFreeSpaceButton();
         ComboBox<ColoringType> coloringTypeButton = ocTreeUIControlFactory.normalBasedColoringButton();
         ToggleButton showEstimatedSurfacesButton = ocTreeUIControlFactory.showEstimatedSurfacesButton();
         ToggleButton showPlanarRegionsButton = ocTreeUIControlFactory.showPlanarRegionsButton();
         ToggleButton hidePlanarRegionNodesButton = ocTreeUIControlFactory.hidePlanarRegionNodesButton();
         ToggleButton advancedStageButton = ocTreeUIControlFactory.showAdvanceStageButton();
         Slider resolutionSlider = ocTreeUIControlFactory.resolutionSlider();
         Label resolutionLabel = new Label("OcTree Resolution: ");
         resolutionLabel.setLabelFor(resolutionSlider);

         add(enableOcTreeViewerButton, 0, row);
         add(clearOcTreeButton, 1, row);
         add(showFreeSpaceButton, 2, row);
         add(resolutionLabel, 3, row);
         add(resolutionSlider, 4, row);
         row++;
         add(advancedStageButton, 0, row);
         add(coloringTypeButton, 1, row);
         add(showEstimatedSurfacesButton, 2, row);
         add(showPlanarRegionsButton, 3, row);
         add(hidePlanarRegionNodesButton, 4, row);
         row++;
         add(ocTreeUIControlFactory.createBoundingBoxPane(), 1, row);
         row++;
         add(ocTreeUIControlFactory.createLidarRangePane(), 1, row);
      }
   }

   public void stop()
   {
      if (pointCloudUIControlFactory != null)
         pointCloudUIControlFactory.stop();
      if (heightMapUIControlFactory != null)
         heightMapUIControlFactory.stop();
      if (ocTreeUIControlFactory != null)
         ocTreeUIControlFactory.stop();
   }
}
