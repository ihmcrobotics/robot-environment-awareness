package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.beans.InvalidationListener;
import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import us.ihmc.octoMap.ocTree.implementations.NormalEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.ui.tools.StringConverterTools;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleAPI;

public class NormalEstimationAnchorPaneController extends REABasicUIController
{
   @FXML
   private Slider searchRadiusSlider;
   @FXML
   private Slider maxDistanceFromPlaneSlider;

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

      InvalidationListener sendParametersListener = observable -> send(new REAMessage(REAModuleAPI.OcTreeNormalEstimationParameters, createNormalEstimationParameters()));
      searchRadiusSlider.valueProperty().addListener(sendParametersListener);
      maxDistanceFromPlaneSlider.valueProperty().addListener(sendParametersListener);
      registerListener(sendParametersListener);
      fireAllListeners();
      load();
   }

   @FXML
   public void save()
   {
      saveProperty(REAModuleAPI.OcTreeNormalEstimationParameters, createNormalEstimationParameters().toString());
   }

   public void load()
   {
      String parameters = loadProperty(REAModuleAPI.OcTreeNormalEstimationParameters);
      if (parameters != null)
      {
         NormalEstimationParameters normalEstimationParameters = NormalEstimationParameters.parse(parameters);
         searchRadiusSlider.setValue(normalEstimationParameters.getSearchRadius());
         maxDistanceFromPlaneSlider.setValue(normalEstimationParameters.getMaxDistanceFromPlane());
      }
   }

   private NormalEstimationParameters createNormalEstimationParameters()
   {
      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setSearchRadius(searchRadiusSlider.getValue());
      normalEstimationParameters.setMaxDistanceFromPlane(maxDistanceFromPlaneSlider.getValue());
      return normalEstimationParameters;
   }
}
