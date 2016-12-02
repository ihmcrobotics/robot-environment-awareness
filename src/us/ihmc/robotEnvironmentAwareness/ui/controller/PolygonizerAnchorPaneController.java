package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.beans.InvalidationListener;
import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;

public class PolygonizerAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enablePolygonizerButton;
   @FXML
   private ToggleButton enableIntersectionCalculatorButton;
   @FXML
   private ToggleButton hideRegionNodes;

   // Polygonizer parameters
   @FXML
   private Spinner<Double> concaveHullThresholdSpinner;
   @FXML
   private Spinner<Integer> minRegionSizePolygonizerSpinner;
   @FXML
   private Spinner<Double> peakAngleThresholdSpinner;
   @FXML
   private Spinner<Double> shallowAngleThresholdSpinner;
   @FXML
   private Spinner<Double> minEdgeLengthSpinner;
   @FXML
   private Spinner<Double> depthThresholdSpinner;

   // Intersection calculator parameters
   @FXML
   private Spinner<Double> maxDistanceToRegionSpinner;
   @FXML
   private Spinner<Integer> minRegionSizeIntersectionSpinner;
   @FXML
   private Spinner<Double> minIntersectionLengthSpinner;
   @FXML
   private Spinner<Double> minRegionAngleDifferenceSpinner;
   @FXML
   private ToggleButton addIntersectionsToRegionsButton;

   public PolygonizerAnchorPaneController()
   {
   }

   private void setupControls()
   {
      concaveHullThresholdSpinner.setValueFactory(createLengthValueFactory(0.001, 0.50, 0.2, 0.05));
      minRegionSizePolygonizerSpinner.setValueFactory(new IntegerSpinnerValueFactory(0, 1000, 10, 10));
      peakAngleThresholdSpinner.setValueFactory(createAngleValueFactory(Math.PI / 2.0, Math.PI, Math.toRadians(160), Math.toRadians(5.0)));
      shallowAngleThresholdSpinner.setValueFactory(createAngleValueFactory(0.0, Math.PI / 2.0, Math.toRadians(10), Math.toRadians(2.5)));
      minEdgeLengthSpinner.setValueFactory(createLengthValueFactory(0.0, 0.20, 0.05, 0.005));
      depthThresholdSpinner.setValueFactory(createLengthValueFactory(0.001, 0.50, 0.10, 0.05));

      maxDistanceToRegionSpinner.setValueFactory(createLengthValueFactory(0.0, 0.50, 0.05, 0.01));
      minRegionSizeIntersectionSpinner.setValueFactory(new IntegerSpinnerValueFactory(0, 1000, 10, 10));
      minIntersectionLengthSpinner.setValueFactory(createLengthValueFactory(0.0, 0.50, 0.06, 0.01));
      minRegionAngleDifferenceSpinner.setValueFactory(createAngleValueFactory(0.0, Math.PI / 2.0, Math.toRadians(15.0), Math.toRadians(5.0)));

      concaveHullThresholdSpinner.getValueFactory().setConverter(StringConverterTools.metersToRoundedCentimeters());
      peakAngleThresholdSpinner.getValueFactory().setConverter(StringConverterTools.radiansToRoundedDegrees());
      shallowAngleThresholdSpinner.getValueFactory().setConverter(StringConverterTools.radiansToRoundedDegrees());
      minEdgeLengthSpinner.getValueFactory().setConverter(StringConverterTools.metersToRoundedCentimeters());
      depthThresholdSpinner.getValueFactory().setConverter(StringConverterTools.metersToRoundedCentimeters());

      maxDistanceToRegionSpinner.getValueFactory().setConverter(StringConverterTools.metersToRoundedCentimeters());
      minIntersectionLengthSpinner.getValueFactory().setConverter(StringConverterTools.metersToRoundedCentimeters());
      minRegionAngleDifferenceSpinner.getValueFactory().setConverter(StringConverterTools.radiansToRoundedDegrees());
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerEnable, enablePolygonizerButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionEnable, enableIntersectionCalculatorButton.selectedProperty());

      uiMessager.bindBidirectionalInternal(REAModuleAPI.OcTreeGraphicsHidePlanarRegionNodes, hideRegionNodes.selectedProperty());
      InvalidationListener sendPolygonizerParametersListener = observable -> uiMessager.submitMessageToModule(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerParameters, createPolygonizerParameters());
      InvalidationListener sendIntersectionParametersListener = observable -> uiMessager.submitMessageToModule(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionParameters, createIntersectionEstimationParameters());

      concaveHullThresholdSpinner.valueProperty().addListener(sendPolygonizerParametersListener);
      minRegionSizePolygonizerSpinner.valueProperty().addListener(sendPolygonizerParametersListener);
      peakAngleThresholdSpinner.valueProperty().addListener(sendPolygonizerParametersListener);
      minEdgeLengthSpinner.valueProperty().addListener(sendPolygonizerParametersListener);
      depthThresholdSpinner.valueProperty().addListener(sendPolygonizerParametersListener);
      shallowAngleThresholdSpinner.valueProperty().addListener(sendPolygonizerParametersListener);

      maxDistanceToRegionSpinner.valueProperty().addListener(sendIntersectionParametersListener);
      minRegionSizeIntersectionSpinner.valueProperty().addListener(sendIntersectionParametersListener);
      minIntersectionLengthSpinner.valueProperty().addListener(sendIntersectionParametersListener);
      minRegionAngleDifferenceSpinner.valueProperty().addListener(sendIntersectionParametersListener);
      addIntersectionsToRegionsButton.selectedProperty().addListener(sendIntersectionParametersListener);

      load();
   }

   @FXML
   public void save()
   {
      saveProperty(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerEnable, enablePolygonizerButton.isSelected());
      saveProperty(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionEnable, enableIntersectionCalculatorButton.isSelected());
      saveProperty(REAModuleAPI.OcTreeGraphicsHidePlanarRegionNodes, hideRegionNodes.isSelected());
      saveProperty(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerParameters, createPolygonizerParameters().toString());
      saveProperty(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionParameters, createIntersectionEstimationParameters().toString());
   }

   private void load()
   {
      loadPropertyAndUpdateUIControl(enablePolygonizerButton, REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerEnable);
      loadPropertyAndUpdateUIControl(enableIntersectionCalculatorButton, REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionEnable);
      loadPropertyAndUpdateUIControl(hideRegionNodes, REAModuleAPI.OcTreeGraphicsHidePlanarRegionNodes);

      String parametersAsString = loadProperty(REAModuleAPI.OcTreePlanarRegionFeaturesPolygonizerParameters);
      if (parametersAsString != null)
      {
         PolygonizerParameters parameters = PolygonizerParameters.parse(parametersAsString);
         concaveHullThresholdSpinner.getValueFactory().setValue(parameters.getConcaveHullThreshold());
         minRegionSizePolygonizerSpinner.getValueFactory().setValue(parameters.getMinNumberOfNodes());
         shallowAngleThresholdSpinner.getValueFactory().setValue(parameters.getShallowAngleThreshold());
         peakAngleThresholdSpinner.getValueFactory().setValue(parameters.getPeakAngleThreshold());
         minEdgeLengthSpinner.getValueFactory().setValue(parameters.getLengthThreshold());
         depthThresholdSpinner.getValueFactory().setValue(parameters.getDepthThreshold());
      }

      parametersAsString = loadProperty(REAModuleAPI.OcTreePlanarRegionFeaturesIntersectionParameters);
      if (parametersAsString != null)
      {
         IntersectionEstimationParameters parameters = IntersectionEstimationParameters.parse(parametersAsString);
         maxDistanceToRegionSpinner.getValueFactory().setValue(parameters.getMaxDistanceToRegion());
         minRegionSizeIntersectionSpinner.getValueFactory().setValue(parameters.getMinRegionSize());
         minIntersectionLengthSpinner.getValueFactory().setValue(parameters.getMinIntersectionLength());
         minRegionAngleDifferenceSpinner.getValueFactory().setValue(parameters.getMinRegionAngleDifference());
         addIntersectionsToRegionsButton.setSelected(parameters.isAddIntersectionsToRegions());
      }
   }

   private PolygonizerParameters createPolygonizerParameters()
   {
      PolygonizerParameters parameters = new PolygonizerParameters();
      parameters.setConcaveHullThreshold(concaveHullThresholdSpinner.getValue());
      parameters.setMinNumberOfNodes(minRegionSizePolygonizerSpinner.getValue());
      parameters.setShallowAngleThreshold(shallowAngleThresholdSpinner.getValue());
      parameters.setPeakAngleThreshold(peakAngleThresholdSpinner.getValue());
      parameters.setLengthThreshold(minEdgeLengthSpinner.getValue());
      parameters.setDepthThreshold(depthThresholdSpinner.getValue());
      return parameters;
   }

   private IntersectionEstimationParameters createIntersectionEstimationParameters()
   {
      IntersectionEstimationParameters parameters = new IntersectionEstimationParameters();
      parameters.setMaxDistanceToRegion(maxDistanceToRegionSpinner.getValue());
      parameters.setMinRegionSize(minRegionSizeIntersectionSpinner.getValue());
      parameters.setMinIntersectionLength(minIntersectionLengthSpinner.getValue());
      parameters.setMinRegionAngleDifference(minRegionAngleDifferenceSpinner.getValue());
      parameters.setAddIntersectionsToRegions(addIntersectionsToRegionsButton.isSelected());
      return parameters;
   }

   private DoubleSpinnerValueFactory createLengthValueFactory(double min, double max, double initialValue, double amountToStepBy)
   {
      DoubleSpinnerValueFactory doubleSpinnerValueFactory = new DoubleSpinnerValueFactory(min, max, initialValue, amountToStepBy);
      doubleSpinnerValueFactory.setConverter(StringConverterTools.metersToRoundedCentimeters());
      return doubleSpinnerValueFactory;
   }
   
   private DoubleSpinnerValueFactory createAngleValueFactory(double min, double max, double initialValue, double amountToStepBy)
   {
      DoubleSpinnerValueFactory doubleSpinnerValueFactory = new DoubleSpinnerValueFactory(min, max, initialValue, amountToStepBy);
      doubleSpinnerValueFactory.setConverter(StringConverterTools.radiansToRoundedDegrees());
      return doubleSpinnerValueFactory;
   }
}
