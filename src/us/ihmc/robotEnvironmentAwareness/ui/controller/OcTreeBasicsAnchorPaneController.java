package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.beans.property.IntegerProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessager;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.updaters.REAOcTreeGraphicsBuilder.ColoringType;

public class OcTreeBasicsAnchorPaneController
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

   private REAMessager outputMessager;
   private final IntegerProperty depthIntegerProperty = new SimpleIntegerProperty(this, "depthInteger");

   public OcTreeBasicsAnchorPaneController()
   {
   }

   public void attachOutputMessager(REAMessager outputMessager)
   {
      this.outputMessager = outputMessager;
   }

   public void setupControls()
   {
      ObservableList<ColoringType> options = FXCollections.observableArrayList(ColoringType.values());
      coloringTypeComboBox.setItems(options);
      coloringTypeComboBox.setValue(options.get(0));
      depthIntegerProperty.bind(depthSlider.valueProperty());
   }

   public void bindControls()
   {
      setupControls();

      sendMessageOnPropertyChange(enableButton.selectedProperty(), REAModuleAPI.OcTreeEnable);
      sendMessageOnPropertyChange(depthIntegerProperty, REAModuleAPI.OcTreeGraphicsDepth);
      sendMessageOnPropertyChange(showOcTreeNodesButton.selectedProperty(), REAModuleAPI.OcTreeGraphicsShowOcTreeNodes);
      sendMessageOnPropertyChange(showEstimatedSurfacesButton.selectedProperty(), REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces);
      sendMessageOnPropertyChange(coloringTypeComboBox.valueProperty(), REAModuleAPI.OcTreeGraphicsColoringMode);
   }

   @FXML
   public void clear()
   {
      send(new REAMessage(REAModuleAPI.OcTreeClear, true));
   }

   private <T> void sendMessageOnPropertyChange(Property<T> property, String messageName)
   {
      ListenerTools.sendMessageOnPropertyChange(property, outputMessager, messageName);
   }

   private void send(REAMessage message)
   {
      outputMessager.submitMessage(message);
   }
}
