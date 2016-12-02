package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import us.ihmc.jOctoMap.boundingBox.OcTreeSimpleBoundingBox;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.properties.BoundingBoxParametersProperty;

public class LIDARFilterAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableBoundingBoxButton;
   @FXML
   private ToggleButton showBoundingBoxButton;
   @FXML
   private Spinner<Double> boundingBoxMinXSpinner;
   @FXML
   private Spinner<Double> boundingBoxMaxXSpinner;
   @FXML
   private Spinner<Double> boundingBoxMinYSpinner;
   @FXML
   private Spinner<Double> boundingBoxMaxYSpinner;
   @FXML
   private Spinner<Double> boundingBoxMinZSpinner;
   @FXML
   private Spinner<Double> boundingBoxMaxZSpinner;

   private final BoundingBoxParametersProperty boundingBoxParametersProperty = new BoundingBoxParametersProperty(this, "boundingBoxParameters");

   @FXML
   private Slider lidarMinRange;
   @FXML
   private Slider lidarMaxRange;

   public LIDARFilterAnchorPaneController()
   {
   }

   private void setupControls()
   {
      boundingBoxMinXSpinner.setValueFactory(createBoundingBoxValueFactory(0.0));
      boundingBoxMaxXSpinner.setValueFactory(createBoundingBoxValueFactory(5.0));
      boundingBoxMinYSpinner.setValueFactory(createBoundingBoxValueFactory(-2.0));
      boundingBoxMaxYSpinner.setValueFactory(createBoundingBoxValueFactory(2.0));
      boundingBoxMinZSpinner.setValueFactory(createBoundingBoxValueFactory(-1.0));
      boundingBoxMaxZSpinner.setValueFactory(createBoundingBoxValueFactory(1.0));
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeBoundingBoxEnable, enableBoundingBoxButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeLIDARMinRange, lidarMinRange.valueProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeLIDARMaxRange, lidarMaxRange.valueProperty());

      boundingBoxParametersProperty.binBidirectionalMinX(boundingBoxMinXSpinner.getValueFactory().valueProperty());
      boundingBoxParametersProperty.binBidirectionalMinY(boundingBoxMinYSpinner.getValueFactory().valueProperty());
      boundingBoxParametersProperty.binBidirectionalMinZ(boundingBoxMinZSpinner.getValueFactory().valueProperty());
      boundingBoxParametersProperty.binBidirectionalMaxX(boundingBoxMaxXSpinner.getValueFactory().valueProperty());
      boundingBoxParametersProperty.binBidirectionalMaxY(boundingBoxMaxYSpinner.getValueFactory().valueProperty());
      boundingBoxParametersProperty.binBidirectionalMaxZ(boundingBoxMaxZSpinner.getValueFactory().valueProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeBoundingBoxParameters, boundingBoxParametersProperty);

      uiMessager.bindBidirectionalInternal(REAModuleAPI.OcTreeGraphicsBoundingBoxShow, showBoundingBoxButton.selectedProperty(), true);

      load();
   }

   @FXML
   public void save()
   {
      saveProperty(REAModuleAPI.OcTreeBoundingBoxEnable, enableBoundingBoxButton.isSelected());
      saveProperty(REAModuleAPI.OcTreeGraphicsBoundingBoxShow, showBoundingBoxButton.isSelected());
      saveProperty(REAModuleAPI.OcTreeLIDARMinRange, lidarMinRange.getValue());
      saveProperty(REAModuleAPI.OcTreeLIDARMaxRange, lidarMaxRange.getValue());
      saveProperty(REAModuleAPI.OcTreeBoundingBoxParameters, createBoundingBox().toString());
   }

   public void load()
   {
      loadPropertyAndUpdateUIControl(enableBoundingBoxButton, REAModuleAPI.OcTreeBoundingBoxEnable);
      loadPropertyAndUpdateUIControl(showBoundingBoxButton, REAModuleAPI.OcTreeGraphicsBoundingBoxShow);
      loadPropertyAndUpdateUIControl(lidarMinRange, REAModuleAPI.OcTreeLIDARMinRange);
      loadPropertyAndUpdateUIControl(lidarMaxRange, REAModuleAPI.OcTreeLIDARMaxRange);
      String boundingBoxAsString = loadProperty(REAModuleAPI.OcTreeBoundingBoxParameters);
      if (boundingBoxAsString != null)
      {
         OcTreeSimpleBoundingBox boundingBox = OcTreeSimpleBoundingBox.parse(boundingBoxAsString);
         updateBoundingBoxSpinners(boundingBox);
      }
   }

   private void updateBoundingBoxSpinners(OcTreeSimpleBoundingBox boundingBox)
   {
      boundingBoxMinXSpinner.getValueFactory().setValue(boundingBox.getMinX());
      boundingBoxMinYSpinner.getValueFactory().setValue(boundingBox.getMinY());
      boundingBoxMinZSpinner.getValueFactory().setValue(boundingBox.getMinZ());
      boundingBoxMaxXSpinner.getValueFactory().setValue(boundingBox.getMaxX());
      boundingBoxMaxYSpinner.getValueFactory().setValue(boundingBox.getMaxY());
      boundingBoxMaxZSpinner.getValueFactory().setValue(boundingBox.getMaxZ());
   }

   private OcTreeSimpleBoundingBox createBoundingBox()
   {
      OcTreeSimpleBoundingBox boundingBox = new OcTreeSimpleBoundingBox();
      boundingBox.setMinCoordinate(boundingBoxMinXSpinner.getValue(), boundingBoxMinYSpinner.getValue(), boundingBoxMinZSpinner.getValue());
      boundingBox.setMaxCoordinate(boundingBoxMaxXSpinner.getValue(), boundingBoxMaxYSpinner.getValue(), boundingBoxMaxZSpinner.getValue());
      return boundingBox;
   }

   private DoubleSpinnerValueFactory createBoundingBoxValueFactory(double initialValue)
   {
      double min = -100.0;
      double max = 100.0;
      double amountToStepBy = 0.1;
      return new DoubleSpinnerValueFactory(min, max, initialValue, amountToStepBy);
   }
}
