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
      depthIntegerProperty.bindBidirectional(depthSlider.valueProperty());
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
   }

   @FXML
   public void clear()
   {
      uiMessager.broadcastMessage(REAModuleAPI.OcTreeClear, true);
   }

   @FXML
   public void save()
   {
      uiMessager.submitStateRequestToModule(REAModuleAPI.SaveMainUpdaterConfiguration);
      uiMessager.submitStateRequestToModule(REAModuleAPI.SaveBufferConfiguration);
   }
}
