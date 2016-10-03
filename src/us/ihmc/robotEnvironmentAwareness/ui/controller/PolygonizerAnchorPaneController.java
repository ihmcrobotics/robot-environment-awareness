package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.ToggleButton;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleAPI;

public class PolygonizerAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableButton;

   @FXML
   private ToggleButton hideRegionNodes;

   public PolygonizerAnchorPaneController()
   {
   }

   @Override
   public void bindControls()
   {
      sendMessageOnPropertyChange(enableButton, REAModuleAPI.OcTreeGraphicsShowPlanarRegions);
      sendMessageOnPropertyChange(hideRegionNodes, REAModuleAPI.OcTreeGraphicsHidePlanarRegionNodes);
      fireAllListeners();

      load();
   }

   @FXML
   public void save()
   {
      saveProperty(REAModuleAPI.OcTreeGraphicsShowPlanarRegions, enableButton.isSelected());
      saveProperty(REAModuleAPI.OcTreeGraphicsHidePlanarRegionNodes, hideRegionNodes.isSelected());
   }

   private void load()
   {
      loadPropertyAndUpdateUIControl(enableButton, REAModuleAPI.OcTreeGraphicsShowPlanarRegions);
      loadPropertyAndUpdateUIControl(hideRegionNodes, REAModuleAPI.OcTreeGraphicsHidePlanarRegionNodes);
   }
}
