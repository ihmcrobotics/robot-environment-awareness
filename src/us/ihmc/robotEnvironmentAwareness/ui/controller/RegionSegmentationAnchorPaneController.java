package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.beans.InvalidationListener;
import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import us.ihmc.octoMap.ocTree.implementations.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.updaters.REAModuleAPI;

public class RegionSegmentationAnchorPaneController extends REABasicUIController
{
   @FXML
   private Slider searchRadiusSlider;
   @FXML
   private Slider maxDistanceFromPlaneSlider;
   @FXML
   private Slider maxAngleFromPlaneSlider;
   @FXML
   private Slider minNormalQualitySlider;

   public RegionSegmentationAnchorPaneController()
   {
   }

   @Override
   public void bindControls()
   {
      InvalidationListener sendParametersListener = observer -> send(new REAMessage(REAModuleAPI.OcTreePlanarRegionSegmentationParameters, createParameters()));
      searchRadiusSlider.valueProperty().addListener(sendParametersListener);
      maxDistanceFromPlaneSlider.valueProperty().addListener(sendParametersListener);
      maxAngleFromPlaneSlider.valueProperty().addListener(sendParametersListener);
      minNormalQualitySlider.valueProperty().addListener(sendParametersListener);
      registerListener(sendParametersListener);
      fireAllListeners();

      load();
   }

   @FXML
   public void save()
   {
      saveProperty(REAModuleAPI.OcTreePlanarRegionSegmentationParameters, createParameters().toString());
   }

   public void load()
   {
      String parametersAsString = loadProperty(REAModuleAPI.OcTreePlanarRegionSegmentationParameters);
      if (parametersAsString != null)
      {
         PlanarRegionSegmentationParameters parameters = PlanarRegionSegmentationParameters.parse(parametersAsString);
         searchRadiusSlider.setValue(parameters.getSearchRadius());
         maxDistanceFromPlaneSlider.setValue(parameters.getMaxDistanceFromPlane());
         maxAngleFromPlaneSlider.setValue(parameters.getMaxAngleFromPlane());
         minNormalQualitySlider.setValue(parameters.getMinNormalQuality());
      }
   }

   private PlanarRegionSegmentationParameters createParameters()
   {
      PlanarRegionSegmentationParameters parameters = new PlanarRegionSegmentationParameters();
      parameters.setSearchRadius(searchRadiusSlider.getValue());
      parameters.setMaxDistanceFromPlane(maxDistanceFromPlaneSlider.getValue());
      parameters.setMaxAngleFromPlane(maxAngleFromPlaneSlider.getValue());
      parameters.setMinNormalQuality(minNormalQualitySlider.getValue());
      return parameters;
   }
}
