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
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.ColoringType;

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
   private final IntegerProperty bufferSizeProperty = new SimpleIntegerProperty(this, "bufferSize");

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
      bufferSizeProperty.bindBidirectional(bufferSizeSlider.valueProperty());
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeEnable, enableButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeBufferSize, bufferSizeProperty);

      uiMessager.bindBidirectionalInternal(REAModuleAPI.OcTreeGraphicsDepth, depthIntegerProperty, true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.OcTreeGraphicsShowOcTreeNodes, showOcTreeNodesButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces, showEstimatedSurfacesButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.OcTreeGraphicsColoringMode, coloringTypeComboBox.valueProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.OcTreeGraphicsShowBuffer, showBufferButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.OcTreeGraphicsShowInputScan, showInputScanButton.selectedProperty(), true);

      load();
   }

   @FXML
   public void clear()
   {
      uiMessager.broadcastMessage(REAModuleAPI.OcTreeClear, true);
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
