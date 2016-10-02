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
      sendMessageOnPropertyChange(depthIntegerProperty, REAModuleAPI.OcTreeGraphicsDepth);
      sendMessageOnPropertyChange(showOcTreeNodesButton, REAModuleAPI.OcTreeGraphicsShowOcTreeNodes);
      sendMessageOnPropertyChange(showEstimatedSurfacesButton, REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces);
      sendMessageOnPropertyChange(coloringTypeComboBox.valueProperty(), REAModuleAPI.OcTreeGraphicsColoringMode);
   }

   @FXML
   public void clear()
   {
      send(new REAMessage(REAModuleAPI.OcTreeClear, true));
   }
}
