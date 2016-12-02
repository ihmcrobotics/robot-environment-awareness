package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.properties.NormalEstimationParametersProperty;

public class NormalEstimationAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableButton;

   @FXML
   private Slider searchRadiusSlider;

   @FXML
   private Slider maxDistanceFromPlaneSlider;

   @FXML
   private Slider minConsensusRatioSlider;

   @FXML
   private Slider maxAverageDeviationRatioSlider;

   private final NormalEstimationParametersProperty normalEstimationParametersProperty = new NormalEstimationParametersProperty(this, "normalEstimationParameters");

   public NormalEstimationAnchorPaneController()
   {
   }

   private void setupControls()
   {
      searchRadiusSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      maxDistanceFromPlaneSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeNormalEstimationEnable, enableButton.selectedProperty());

      normalEstimationParametersProperty.bindBidirectionalSearchRadius(searchRadiusSlider.valueProperty());
      normalEstimationParametersProperty.bindBidirectionalMaxDistanceFromPlane(maxDistanceFromPlaneSlider.valueProperty());
      normalEstimationParametersProperty.bindBidirectionalMinConsensusRatio(minConsensusRatioSlider.valueProperty());
      normalEstimationParametersProperty.bindBidirectionalMaxAverageDeviationRatio(maxAverageDeviationRatioSlider.valueProperty());

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeNormalEstimationParameters, normalEstimationParametersProperty);

      load();
   }

   @FXML
   public void save()
   {
      saveProperty(REAModuleAPI.OcTreeNormalEstimationEnable, enableButton.isSelected());
      saveProperty(REAModuleAPI.OcTreeNormalEstimationParameters, createNormalEstimationParameters().toString());
   }

   public void load()
   {
      loadPropertyAndUpdateUIControl(enableButton, REAModuleAPI.OcTreeNormalEstimationEnable);
      String parameters = loadProperty(REAModuleAPI.OcTreeNormalEstimationParameters);
      if (parameters != null)
      {
         NormalEstimationParameters normalEstimationParameters = NormalEstimationParameters.parse(parameters);
         searchRadiusSlider.setValue(normalEstimationParameters.getSearchRadius());
         maxDistanceFromPlaneSlider.setValue(normalEstimationParameters.getMaxDistanceFromPlane());
         minConsensusRatioSlider.setValue(normalEstimationParameters.getMinConsensusRatio());
         maxAverageDeviationRatioSlider.setValue(normalEstimationParameters.getMaxAverageDeviationRatio());
      }
   }

   @FXML
   public void resetNormals()
   {
      uiMessager.broadcastMessage(REAModuleAPI.OcTreeNormalEstimationClear, true);
   }

   private NormalEstimationParameters createNormalEstimationParameters()
   {
      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setSearchRadius(searchRadiusSlider.getValue());
      normalEstimationParameters.setMaxDistanceFromPlane(maxDistanceFromPlaneSlider.getValue());
      normalEstimationParameters.setMinConsensusRatio(minConsensusRatioSlider.getValue());
      normalEstimationParameters.setMaxAverageDeviationRatio(maxAverageDeviationRatioSlider.getValue());
      return normalEstimationParameters;
   }
}
