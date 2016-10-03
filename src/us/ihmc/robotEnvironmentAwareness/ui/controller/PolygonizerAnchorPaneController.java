package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.ToggleButton;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleAPI;

public class PolygonizerAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enablePolygonizerButton;
   
   @FXML
   private ToggleButton enableIntersectionCalculatorButton;

   @FXML
   private ToggleButton hideRegionNodes;

   public PolygonizerAnchorPaneController()
   {
   }

   @Override
   public void bindControls()
   {
      sendMessageOnPropertyChange(enablePolygonizerButton, REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerEnable);
      sendMessageOnPropertyChange(enableIntersectionCalculatorButton, REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionEnable);
      sendMessageOnPropertyChange(hideRegionNodes, REAModuleAPI.OcTreeGraphicsHidePlanarRegionNodes);
      fireAllListeners();

      load();
   }

   @FXML
   public void save()
   {
      saveProperty(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerEnable, enablePolygonizerButton.isSelected());
      saveProperty(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionEnable, enableIntersectionCalculatorButton.isSelected());
      saveProperty(REAModuleAPI.OcTreeGraphicsHidePlanarRegionNodes, hideRegionNodes.isSelected());
   }

   private void load()
   {
      loadPropertyAndUpdateUIControl(enablePolygonizerButton, REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerEnable);
      loadPropertyAndUpdateUIControl(enableIntersectionCalculatorButton, REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionEnable);
      loadPropertyAndUpdateUIControl(hideRegionNodes, REAModuleAPI.OcTreeGraphicsHidePlanarRegionNodes);
   }
}
