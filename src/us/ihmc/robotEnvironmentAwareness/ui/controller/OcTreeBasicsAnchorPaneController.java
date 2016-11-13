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
import us.ihmc.javaFXToolkit.StringConverterTools;
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
   @FXML
   private ToggleButton showBufferButton;
   @FXML
   private Slider bufferSizeSlider;
   @FXML
   private ToggleButton showInputScanButton;

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
      bufferSizeSlider.setLabelFormatter(StringConverterTools.thousandRounding(true));
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
      sendMessageOnPropertyChange(showBufferButton, REAModuleAPI.OcTreeGraphicsShowBuffer);
      sendMessageOnPropertyChange(bufferSizeSlider, REAModuleAPI.OcTreeBufferSize);
      sendMessageOnPropertyChange(showInputScanButton, REAModuleAPI.OcTreeGraphicsShowInputScan);
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
      loadPropertyAndUpdateUIControl(depthSlider, REAModuleAPI.OcTreeGraphicsDepth);
      loadPropertyAndUpdateUIControl(showOcTreeNodesButton, REAModuleAPI.OcTreeGraphicsShowOcTreeNodes);
      loadPropertyAndUpdateUIControl(showEstimatedSurfacesButton, REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces);
      loadPropertyAndUpdateUIControl(coloringTypeComboBox, REAModuleAPI.OcTreeGraphicsColoringMode);
      loadPropertyAndUpdateUIControl(showBufferButton, REAModuleAPI.OcTreeGraphicsShowBuffer);
      loadPropertyAndUpdateUIControl(bufferSizeSlider, REAModuleAPI.OcTreeBufferSize);
      loadPropertyAndUpdateUIControl(showInputScanButton, REAModuleAPI.OcTreeGraphicsShowInputScan);
   }

   @FXML
   public void save()
   {
      saveProperty(REAModuleAPI.OcTreeEnable, enableButton.isSelected());
      saveProperty(REAModuleAPI.OcTreeGraphicsDepth, depthIntegerProperty.intValue());
      saveProperty(REAModuleAPI.OcTreeGraphicsShowOcTreeNodes, showOcTreeNodesButton.isSelected());
      saveProperty(REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces, showEstimatedSurfacesButton.isSelected());
      saveProperty(REAModuleAPI.OcTreeGraphicsColoringMode, coloringTypeComboBox.getValue().toString());
      saveProperty(REAModuleAPI.OcTreeGraphicsShowBuffer, showBufferButton.isSelected());
      saveProperty(REAModuleAPI.OcTreeBufferSize, bufferSizeSlider.getValue());
      saveProperty(REAModuleAPI.OcTreeGraphicsShowInputScan, showInputScanButton.isSelected());
   }
}
