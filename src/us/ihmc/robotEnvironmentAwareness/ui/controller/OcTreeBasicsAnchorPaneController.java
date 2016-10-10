package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.updaters.REAOcTreeGraphicsBuilder.ColoringType;

public class OcTreeBasicsAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableButton;
   @FXML
   private ToggleButton enableNormalEstimationButton;
   @FXML
   private Button clearButton;
   @FXML
   private Slider depthSlider;
   @FXML
   private ToggleButton showOcTreeNodesButton;
   @FXML
   private ToggleButton showEstimatedSurfacesButton;
   @FXML
   private ComboBox<ColoringType> coloringTypeComboBox;

   private final IntegerProperty depthIntegerProperty = new SimpleIntegerProperty(this, "depthInteger");

   public OcTreeBasicsAnchorPaneController()
   {
   }

   public void setupControls()
   {
      ObservableList<ColoringType> options = FXCollections.observableArrayList(ColoringType.values());
      coloringTypeComboBox.setItems(options);
      coloringTypeComboBox.setValue(options.get(0));
      depthIntegerProperty.bind(depthSlider.valueProperty());
   }

   @Override
   public void bindControls()
   {
      setupControls();

      sendMessageOnPropertyChange(enableButton, REAModuleAPI.OcTreeEnable);
      sendMessageOnPropertyChange(enableNormalEstimationButton, REAModuleAPI.OcTreeNormalEstimationEnable);
      sendMessageOnPropertyChange(depthIntegerProperty, REAModuleAPI.OcTreeGraphicsDepth);
      sendMessageOnPropertyChange(showOcTreeNodesButton, REAModuleAPI.OcTreeGraphicsShowOcTreeNodes);
      sendMessageOnPropertyChange(showEstimatedSurfacesButton, REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces);
      sendMessageOnPropertyChange(coloringTypeComboBox.valueProperty(), REAModuleAPI.OcTreeGraphicsColoringMode);
      fireAllListeners();

      load();
   }

   @FXML
   public void clear()
   {
      send(new REAMessage(REAModuleAPI.OcTreeClear, true));
   }

   public void load()
   {
      loadPropertyAndUpdateUIControl(enableButton, REAModuleAPI.OcTreeEnable);
      loadPropertyAndUpdateUIControl(enableNormalEstimationButton, REAModuleAPI.OcTreeNormalEstimationEnable);
      loadPropertyAndUpdateUIControl(depthSlider, REAModuleAPI.OcTreeGraphicsDepth);
      loadPropertyAndUpdateUIControl(showOcTreeNodesButton, REAModuleAPI.OcTreeGraphicsShowOcTreeNodes);
      loadPropertyAndUpdateUIControl(showEstimatedSurfacesButton, REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces);
      loadPropertyAndUpdateUIControl(coloringTypeComboBox, REAModuleAPI.OcTreeGraphicsColoringMode);
   }

   @FXML
   public void save()
   {
      saveProperty(REAModuleAPI.OcTreeEnable, enableButton.isSelected());
      saveProperty(REAModuleAPI.OcTreeNormalEstimationEnable, enableNormalEstimationButton.isSelected());
      saveProperty(REAModuleAPI.OcTreeGraphicsDepth, depthIntegerProperty.intValue());
      saveProperty(REAModuleAPI.OcTreeGraphicsShowOcTreeNodes, showOcTreeNodesButton.isSelected());
      saveProperty(REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces, showEstimatedSurfacesButton.isSelected());
      saveProperty(REAModuleAPI.OcTreeGraphicsColoringMode, coloringTypeComboBox.getValue().toString());
   }
}
