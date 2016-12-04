package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.robotEnvironmentAwareness.communication.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
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

   private final PropertyToMessageTypeConverter<Integer, Number> numberToIntegerConverter = new PropertyToMessageTypeConverter<Integer, Number>()
   {
      @Override
      public Integer convert(Number propertyValue)
      {
         return propertyValue.intValue();
      }

      @Override
      public Number interpret(Integer newValue)
      {
         return new Double(newValue.doubleValue());
      }
   };

   public OcTreeBasicsAnchorPaneController()
   {
   }

   public void setupControls()
   {
      ObservableList<ColoringType> options = FXCollections.observableArrayList(ColoringType.values());
      coloringTypeComboBox.setItems(options);
      coloringTypeComboBox.setValue(options.get(ColoringType.REGION.ordinal()));
      bufferSizeSlider.setLabelFormatter(StringConverterTools.thousandRounding(true));
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeEnable, enableButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeBufferSize, bufferSizeSlider.valueProperty(), numberToIntegerConverter);

      uiMessager.bindBidirectionalInternal(REAModuleAPI.OcTreeGraphicsDepth, depthSlider.valueProperty(), numberToIntegerConverter);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.OcTreeGraphicsShowOcTreeNodes, showOcTreeNodesButton.selectedProperty());
      uiMessager.bindBidirectionalInternal(REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces, showEstimatedSurfacesButton.selectedProperty());
      uiMessager.bindBidirectionalInternal(REAModuleAPI.OcTreeGraphicsColoringMode, coloringTypeComboBox.valueProperty());
      uiMessager.bindBidirectionalInternal(REAModuleAPI.OcTreeGraphicsShowBuffer, showBufferButton.selectedProperty());
      uiMessager.bindBidirectionalInternal(REAModuleAPI.OcTreeGraphicsShowInputScan, showInputScanButton.selectedProperty());
      load();
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

      saveUIControlProperty(REAModuleAPI.OcTreeGraphicsDepth, depthSlider);
      saveUIControlProperty(REAModuleAPI.OcTreeGraphicsShowOcTreeNodes, showOcTreeNodesButton);
      saveUIControlProperty(REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces, showEstimatedSurfacesButton);
      saveUIControlProperty(REAModuleAPI.OcTreeGraphicsColoringMode, coloringTypeComboBox);
      saveUIControlProperty(REAModuleAPI.OcTreeGraphicsShowBuffer, showBufferButton);
      saveUIControlProperty(REAModuleAPI.OcTreeGraphicsShowInputScan, showInputScanButton);
   }

   public void load()
   {
      loadUIControlProperty(REAModuleAPI.OcTreeGraphicsDepth, depthSlider);
      loadUIControlProperty(REAModuleAPI.OcTreeGraphicsShowOcTreeNodes, showOcTreeNodesButton);
      loadUIControlProperty(REAModuleAPI.OcTreeGraphicsShowEstimatedSurfaces, showEstimatedSurfacesButton);
      loadUIControlProperty(REAModuleAPI.OcTreeGraphicsColoringMode, coloringTypeComboBox);
      loadUIControlProperty(REAModuleAPI.OcTreeGraphicsShowBuffer, showBufferButton);
      loadUIControlProperty(REAModuleAPI.OcTreeGraphicsShowInputScan, showInputScanButton);
   }
}
