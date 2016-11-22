package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.beans.InvalidationListener;
import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;

public class RegionSegmentationAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableSegmentationButton;
   @FXML
   private Slider searchRadiusSlider;
   @FXML
   private Slider maxDistanceFromPlaneSlider;
   @FXML
   private Slider maxAngleFromPlaneSlider;
   @FXML
   private Slider minNormalQualitySlider;
   @FXML
   private Slider minRegionSizeSlider;

   public RegionSegmentationAnchorPaneController()
   {
   }

   private void setupControls()
   {
      searchRadiusSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      maxDistanceFromPlaneSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      maxAngleFromPlaneSlider.setLabelFormatter(StringConverterTools.radiansToRoundedDegrees());
      minNormalQualitySlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
   }

   @Override
   public void bindControls()
   {
      setupControls();

      sendMessageOnPropertyChange(enableSegmentationButton, REAModuleAPI.OcTreePlanarRegionSegmentationEnable);
      InvalidationListener sendParametersListener = observer -> send(new REAMessage(REAModuleAPI.OcTreePlanarRegionSegmentationParameters, createParameters()));
      searchRadiusSlider.valueProperty().addListener(sendParametersListener);
      maxDistanceFromPlaneSlider.valueProperty().addListener(sendParametersListener);
      maxAngleFromPlaneSlider.valueProperty().addListener(sendParametersListener);
      minNormalQualitySlider.valueProperty().addListener(sendParametersListener);
      minRegionSizeSlider.valueProperty().addListener(sendParametersListener);
      registerListener(sendParametersListener);
      fireAllListeners();

      load();
   }

   @FXML
   public void save()
   {
      saveProperty(REAModuleAPI.OcTreePlanarRegionSegmentationEnable, enableSegmentationButton.isSelected());
      saveProperty(REAModuleAPI.OcTreePlanarRegionSegmentationParameters, createParameters().toString());
   }

   public void load()
   {
      loadPropertyAndUpdateUIControl(enableSegmentationButton, REAModuleAPI.OcTreePlanarRegionSegmentationEnable);

      String parametersAsString = loadProperty(REAModuleAPI.OcTreePlanarRegionSegmentationParameters);
      if (parametersAsString != null)
      {
         PlanarRegionSegmentationParameters parameters = PlanarRegionSegmentationParameters.parse(parametersAsString);
         searchRadiusSlider.setValue(parameters.getSearchRadius());
         maxDistanceFromPlaneSlider.setValue(parameters.getMaxDistanceFromPlane());
         maxAngleFromPlaneSlider.setValue(parameters.getMaxAngleFromPlane());
         minNormalQualitySlider.setValue(parameters.getMinNormalQuality());
         minRegionSizeSlider.setValue(parameters.getMinRegionSize());
      }
   }

   @FXML
   public void clear()
   {
      send(new REAMessage(REAModuleAPI.OcTreePlanarRegionSegmentationClear, true));
   }

   private PlanarRegionSegmentationParameters createParameters()
   {
      PlanarRegionSegmentationParameters parameters = new PlanarRegionSegmentationParameters();
      parameters.setSearchRadius(searchRadiusSlider.getValue());
      parameters.setMaxDistanceFromPlane(maxDistanceFromPlaneSlider.getValue());
      parameters.setMaxAngleFromPlane(maxAngleFromPlaneSlider.getValue());
      parameters.setMinNormalQuality(minNormalQualitySlider.getValue());
      parameters.setMinRegionSize((int) minRegionSizeSlider.getValue());
      return parameters;
   }
}
