package us.ihmc.robotEnvironmentAwareness.ui.ocTree;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Orientation;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Pane;
import javafx.scene.text.Text;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import us.ihmc.octoMap.ocTree.baseImplementation.OcTreeBoundingBox;
import us.ihmc.octoMap.occupancy.OccupancyParameters;
import us.ihmc.robotEnvironmentAwareness.ui.ocTree.OcTreeGraphicsBuilder.ColoringType;
import us.ihmc.robotics.geometry.Direction;

public class OcTreeUIControlFactory
{
   private final OcTreeViewer ocTreeViewer;

   public OcTreeUIControlFactory(OcTreeViewer ocTreeViewer)
   {
      this.ocTreeViewer = ocTreeViewer;
   }

   public Button clearOcTreeButton()
   {
      final BooleanProperty clearProperty = ocTreeViewer.clearOcTreeProperty();
      Button clearOcTreeButton = new Button("Clear octree");
      clearOcTreeButton.setOnAction(new EventHandler<ActionEvent>()
      {
         @Override
         public void handle(ActionEvent event)
         {
            clearProperty.set(true);
         }
      });

      return clearOcTreeButton;
   }

   public ToggleButton enableOcTreeViewerButton()
   {
      final BooleanProperty enableProperty = ocTreeViewer.enableProperty();
      final ToggleButton enableViewerButton = new ToggleButton("Enable octree viewer");
      enableViewerButton.setSelected(enableProperty.get());
      enableViewerButton.selectedProperty().bindBidirectional(enableProperty);

      return enableViewerButton;
   }

   public Slider resolutionSlider()
   {
      return resolutionSlider(Orientation.HORIZONTAL);
   }

   public Slider resolutionSlider(Orientation orientation)
   {
      final IntegerProperty viewerDepthProperty = ocTreeViewer.viewerDepthProperty();
      Slider viewerDepthSlider = new Slider(2, ocTreeViewer.getMaximumTreeDepth(), viewerDepthProperty.intValue());
      viewerDepthSlider.setOrientation(orientation);
      viewerDepthSlider.setShowTickLabels(true);
      viewerDepthSlider.setShowTickMarks(true);
      viewerDepthSlider.setMajorTickUnit(1);
      viewerDepthSlider.setMinorTickCount(1);
      viewerDepthSlider.setBlockIncrement(1);
      viewerDepthSlider.setPrefWidth(200);

      viewerDepthSlider.valueProperty().bindBidirectional(viewerDepthProperty);

      return viewerDepthSlider;
   }

   public ToggleButton showFreeSpaceButton()
   {
      final BooleanProperty showFreeSpaceProperty = ocTreeViewer.showFreeSpaceProperty();
      final ToggleButton showFreeSpaceButton = new ToggleButton("Show free space");
      showFreeSpaceButton.setSelected(showFreeSpaceProperty.get());
      showFreeSpaceButton.selectedProperty().bindBidirectional(showFreeSpaceProperty);

      return showFreeSpaceButton;
   }

   public ComboBox<ColoringType> normalBasedColoringButton()
   {
      final ObjectProperty<ColoringType> coloringTypeProperty = ocTreeViewer.coloringTypeProperty();
      ObservableList<ColoringType> options = FXCollections.observableArrayList(ColoringType.values());
      ComboBox<ColoringType> coloringTypeComboBox = new ComboBox<>(options);
      coloringTypeComboBox.setValue(coloringTypeProperty.get());
      coloringTypeComboBox.setOnAction(new EventHandler<ActionEvent>()
      {
         @Override
         public void handle(ActionEvent event)
         {
            coloringTypeProperty.set(coloringTypeComboBox.getValue());
         }
      });

      return coloringTypeComboBox;
   }

   public ToggleButton showEstimatedSurfacesButton()
   {
      final BooleanProperty showEstimatedSurfacesProperty = ocTreeViewer.showEstimatedSurfacesProperty();
      final ToggleButton showEstimatedSurfacesButton = new ToggleButton("Show estimated surfaces");
      showEstimatedSurfacesButton.setSelected(showEstimatedSurfacesProperty.get());
      showEstimatedSurfacesButton.selectedProperty().bindBidirectional(showEstimatedSurfacesProperty);

      return showEstimatedSurfacesButton;
   }

   private static final double SLIDER_MIN_PROBABILITY = 0.01;
   private static final double SLIDER_MAX_PROBABILITY = 0.99;

   public Slider occupancyThresholdSlider(Orientation orientation)
   {
      return ocTreeProbabilityParameterSlider(orientation, ocTreeViewer.occupancyThresholdProperty());
   }

   public Slider hitUpdateSlider(Orientation orientation)
   {
      return ocTreeProbabilityParameterSlider(orientation, ocTreeViewer.hitUpdateProperty());
   }

   public Slider missUpdateSlider(Orientation orientation)
   {
      return ocTreeProbabilityParameterSlider(orientation, ocTreeViewer.missUpdateProperty());
   }

   public Slider minProbabilitySlider(Orientation orientation)
   {
      return ocTreeProbabilityParameterSlider(orientation, ocTreeViewer.minProbabilityProperty());
   }

   public Slider maxProbabilitySlider(Orientation orientation)
   {
      return ocTreeProbabilityParameterSlider(orientation, ocTreeViewer.maxProbabilityProperty());
   }

   private Slider ocTreeProbabilityParameterSlider(Orientation orientation, DoubleProperty property)
   {
      Slider propertySlider = new Slider(SLIDER_MIN_PROBABILITY, SLIDER_MAX_PROBABILITY, property.doubleValue());
      propertySlider.setOrientation(orientation);
      propertySlider.setShowTickLabels(true);
      propertySlider.setShowTickMarks(true);
      propertySlider.setMajorTickUnit(0.20);
      propertySlider.setBlockIncrement(0.01);
      if (orientation == Orientation.VERTICAL)
         propertySlider.setPrefHeight(400);

      propertySlider.valueProperty().bindBidirectional(property);

      return propertySlider;
   }

   public Button resetOccupancyThresholdButton()
   {
      return ocTreeResetParameterButton("Reset occupancy threshold", ocTreeViewer.occupancyThresholdProperty(), OccupancyParameters.DEFAULT_OCCUPANCY_THRESHOLD);
   }

   public Button resetHitUpdateButton()
   {
      return ocTreeResetParameterButton("Reset hit update", ocTreeViewer.hitUpdateProperty(), OccupancyParameters.DEFAULT_HIT_UPDATE);
   }

   public Button resetMissUpdateButton()
   {
      return ocTreeResetParameterButton("Reset miss update", ocTreeViewer.missUpdateProperty(), OccupancyParameters.DEFAULT_MISS_UPDATE);
   }

   public Button resetMinProbabilityButton()
   {
      return ocTreeResetParameterButton("Reset min probability", ocTreeViewer.minProbabilityProperty(), OccupancyParameters.DEFAULT_MIN_PROBABILITY);
   }

   public Button resetMaxProbabilityButton()
   {
      return ocTreeResetParameterButton("Reset max probability", ocTreeViewer.maxProbabilityProperty(), OccupancyParameters.DEFAULT_MAX_PROBABILITY);
   }

   private Button ocTreeResetParameterButton(String name, DoubleProperty property, double defaultValue)
   {
      Button propertyButton = new Button(name);
      propertyButton.setOnAction(new EventHandler<ActionEvent>()
      {
         @Override
         public void handle(ActionEvent event)
         {
            property.set(defaultValue);
         }
      });

      return propertyButton;
   }

   public Text occupancyThresholdText()
   {
      return ocTreeDisplayParameterText(ocTreeViewer.occupancyThresholdProperty());
   }

   public Text hitUpdateText()
   {
      return ocTreeDisplayParameterText(ocTreeViewer.hitUpdateProperty());
   }

   public Text missUpdateText()
   {
      return ocTreeDisplayParameterText(ocTreeViewer.missUpdateProperty());
   }

   public Text minProbabilityText()
   {
      return ocTreeDisplayParameterText(ocTreeViewer.minProbabilityProperty());
   }

   public Text maxProbabilityText()
   {
      return ocTreeDisplayParameterText(ocTreeViewer.maxProbabilityProperty());
   }

   private Text ocTreeDisplayParameterText(DoubleProperty property)
   {
      NumberFormat format = new DecimalFormat("0.00");
      Text propertyText = new Text("Value: " + format.format(property.doubleValue()));
      property.addListener(new ChangeListener<Number>()
      {
         @Override
         public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue)
         {
            if (Double.isNaN(newValue.doubleValue()))
               return;

            propertyText.setText("Value: " + format.format(property.doubleValue()));
         }
      });
      return propertyText;
   }

   public ToggleButton showAdvanceStageButton()
   {
      ToggleButton showAdvancedStage = new ToggleButton("Advanced UI");
      showAdvancedStage.setSelected(false);
      Stage advancedStage = advancedStage();
      advancedStage.setOnCloseRequest(new EventHandler<WindowEvent>()
      {
         @Override
         public void handle(WindowEvent event)
         {
            showAdvancedStage.setSelected(false);
         }
      });
      listeners.add(new Stoppable()
      {
         @Override
         public void stop()
         {
            advancedStage.close();
         }
      });
      showAdvancedStage.setOnAction(new EventHandler<ActionEvent>()
      {
         @Override
         public void handle(ActionEvent event)
         {
            if (showAdvancedStage.isSelected())
            {
               if (!advancedStage.isShowing())
                  advancedStage.show();
            }
            else
            {
               if (advancedStage.isShowing())
                  advancedStage.close();
            }
         }
      });

      return showAdvancedStage;
   }

   public Pane createBoundingBoxPane()
   {
      double min = -100.0;
      double max = 100.0;
      int preferredSpinnerWidth = 70;

      GridPane gridPane = new GridPane();

      int row = 0;

      Label boundingBoxLabel = new Label("Bounding Box");
      gridPane.add(boundingBoxLabel, 0, row);
      row++;

      BooleanProperty enableProperty = ocTreeViewer.enableBoundingBoxProperty();
      ToggleButton enableBoundingBoxButton = new ToggleButton("Enable");
      enableBoundingBoxButton.setSelected(enableProperty.get());
      enableBoundingBoxButton.selectedProperty().bindBidirectional(enableProperty);
      gridPane.add(enableBoundingBoxButton, 0, row);


      ObjectProperty<OcTreeBoundingBox> boundingBoxProperty = ocTreeViewer.boundingBoxProperty();
      double[] initialMinValue = new double[3];
      double[] initialMaxValue = new double[3];
      boundingBoxProperty.get().getMinCoordinate(initialMinValue);
      boundingBoxProperty.get().getMaxCoordinate(initialMaxValue);

      for (int i = 0; i < 3; i++)
      {
         int index = i;
         SpinnerValueFactory.DoubleSpinnerValueFactory minSpinnerValueFactory = new SpinnerValueFactory.DoubleSpinnerValueFactory(min, max, initialMinValue[index], 0.1);
         minSpinnerValueFactory.valueProperty().addListener(new ChangeListener<Double>()
         {
            @Override
            public void changed(ObservableValue<? extends Double> observable, Double oldValue, Double newValue)
            {
               double[] minPoint = new double[3];
               double[] maxPoint = new double[3];

               if (!newValue.isNaN())
               {
                  OcTreeBoundingBox oldBoundingBox = boundingBoxProperty.get();
                  oldBoundingBox.getMinCoordinate(minPoint);
                  oldBoundingBox.getMaxCoordinate(maxPoint);
                  minPoint[index] = newValue;
                  boundingBoxProperty.set(new OcTreeBoundingBox(minPoint, maxPoint));
               }
            }
         });

         SpinnerValueFactory.DoubleSpinnerValueFactory maxSpinnerValueFactory = new SpinnerValueFactory.DoubleSpinnerValueFactory(min, max, initialMaxValue[index], 0.1);
         maxSpinnerValueFactory.valueProperty().addListener(new ChangeListener<Double>()
         {
            @Override
            public void changed(ObservableValue<? extends Double> observable, Double oldValue, Double newValue)
            {
               double[] minPoint = new double[3];
               double[] maxPoint = new double[3];

               if (!newValue.isNaN())
               {
                  OcTreeBoundingBox oldBoundingBox = boundingBoxProperty.get();
                  oldBoundingBox.getMinCoordinate(minPoint);
                  oldBoundingBox.getMaxCoordinate(maxPoint);
                  maxPoint[index] = newValue;
                  boundingBoxProperty.set(new OcTreeBoundingBox(minPoint, maxPoint));
               }
            }
         });
         

         Spinner<Double> minSpinner = new Spinner<>(minSpinnerValueFactory);
         Spinner<Double> maxSpinner = new Spinner<>(maxSpinnerValueFactory);
         minSpinner.setPrefWidth(preferredSpinnerWidth);
         maxSpinner.setPrefWidth(preferredSpinnerWidth);
         minSpinner.setEditable(true);
         maxSpinner.setEditable(true);

         Label minAxisLabel = new Label(" min" + Direction.values[i].toString() + ": ");
         Label maxAxisLabel = new Label(" max" + Direction.values[i].toString() + ": ");
         gridPane.add(minAxisLabel, 2 * index + 1, row);
         gridPane.add(maxAxisLabel, 2 * index + 1, row + 1);
         gridPane.add(minSpinner, 2 * index + 2, row);
         gridPane.add(maxSpinner, 2 * index + 2, row + 1);
      }
      return gridPane;
   }

   public Pane createLidarRangePane()
   {
      DoubleProperty minRangeProperty = ocTreeViewer.ocTreeMinRangeProperty();
      DoubleProperty maxRangeProperty = ocTreeViewer.ocTreeMaxRangeProperty();
      
      GridPane gridPane = new GridPane();
      int row = 0;
      Label label = new Label("LIDAR Range: ");
      gridPane.add(label, 0, row);

      row++;
      
      Label minRangeLabel = new Label("minRange: ");
      gridPane.add(minRangeLabel, 0, row);

      Slider minRangeSlider = new Slider(0.0, 1.0, minRangeProperty.doubleValue());
      minRangeSlider.setOrientation(Orientation.HORIZONTAL);
      minRangeSlider.setShowTickLabels(true);
      minRangeSlider.setShowTickMarks(true);
      minRangeSlider.setMajorTickUnit(0.20);
      minRangeSlider.setBlockIncrement(0.02);
      minRangeSlider.setPrefWidth(300.0);
      minRangeSlider.valueProperty().bindBidirectional(minRangeProperty);
      gridPane.add(minRangeSlider, 1, row);

      row++;
      
      Label maxRangeLabel = new Label("maxRange: ");
      gridPane.add(maxRangeLabel, 0, row);

      Slider maxRangeSlider = new Slider(1.0, 15.0, maxRangeProperty.doubleValue());
      maxRangeSlider.setOrientation(Orientation.HORIZONTAL);
      maxRangeSlider.setShowTickLabels(true);
      maxRangeSlider.setShowTickMarks(true);
      maxRangeSlider.setMajorTickUnit(1.00);
      maxRangeSlider.setBlockIncrement(0.5);
      maxRangeSlider.setPrefWidth(300.0);
      maxRangeSlider.valueProperty().bindBidirectional(maxRangeProperty);
      gridPane.add(maxRangeSlider, 1, row);


      return gridPane;
   }

   private Stage advancedStage()
   {
      Stage advanced = new Stage();
      advanced.setTitle("Advanced OcTree Window");

      Group root = new Group();
      Scene scene = new Scene(root, 800, 800);

      GridPane gridPane = new GridPane();

      int row = 0;
      int col = 0;

      gridPane.add(enableOcTreeViewerButton(), col++, row);
      gridPane.add(clearOcTreeButton(), col++, row);
      gridPane.add(showFreeSpaceButton(), col++, row);
      gridPane.add(normalBasedColoringButton(), col++, row);
      gridPane.add(showEstimatedSurfacesButton(), col++, row);

      col = 0;
      row++;

      Label parametersLabel = new Label("OcTree Parameters: ");
      gridPane.add(parametersLabel, col, row);

      row++;
      col = 0;

      Label resolutionLabel = new Label("Resolution: ");
      Label occupancyThresholdLabel = new Label("Occupancy Threshold: ");
      Label hitUpdateLabel = new Label("Hit Update: ");
      Label missUpdateLabel = new Label("Miss Update: ");
      Label maxProbabilityLabel = new Label("Max Probability: ");
      Label minProbabilityLabel = new Label("Min Probability: ");

      gridPane.add(resolutionLabel, col++, row);
      gridPane.add(occupancyThresholdLabel, col++, row);
      gridPane.add(hitUpdateLabel, col++, row);
      gridPane.add(missUpdateLabel, col++, row);
      gridPane.add(minProbabilityLabel, col++, row);
      gridPane.add(maxProbabilityLabel, col++, row);

      row++;
      col = 1;

      gridPane.add(occupancyThresholdText(), col++, row);
      gridPane.add(hitUpdateText(), col++, row);
      gridPane.add(missUpdateText(), col++, row);
      gridPane.add(minProbabilityText(), col++, row);
      gridPane.add(maxProbabilityText(), col++, row);

      row++;
      col = 0;

      Slider resolutionSlider = resolutionSlider(Orientation.VERTICAL);
      Slider occupancyThresholdSlider = occupancyThresholdSlider(Orientation.VERTICAL);
      Slider hitUpdateSlider = hitUpdateSlider(Orientation.VERTICAL);
      Slider missUpdateSlider = missUpdateSlider(Orientation.VERTICAL);
      Slider minProbabilitySlider = minProbabilitySlider(Orientation.VERTICAL);
      Slider maxProbabilitySlider = maxProbabilitySlider(Orientation.VERTICAL);
      resolutionLabel.setLabelFor(resolutionSlider);
      occupancyThresholdLabel.setLabelFor(occupancyThresholdSlider);
      hitUpdateLabel.setLabelFor(hitUpdateSlider);
      missUpdateLabel.setLabelFor(missUpdateSlider);
      maxProbabilityLabel.setLabelFor(minProbabilitySlider);
      minProbabilityLabel.setLabelFor(maxProbabilitySlider);

      gridPane.add(resolutionSlider, col++, row);
      gridPane.add(occupancyThresholdSlider, col++, row);
      gridPane.add(hitUpdateSlider, col++, row);
      gridPane.add(missUpdateSlider, col++, row);
      gridPane.add(minProbabilitySlider, col++, row);
      gridPane.add(maxProbabilitySlider, col++, row);

      row++;
      row++;
      col = 1;

      gridPane.add(resetOccupancyThresholdButton(), col++, row);
      gridPane.add(resetHitUpdateButton(), col++, row);
      gridPane.add(resetMissUpdateButton(), col++, row);
      gridPane.add(resetMinProbabilityButton(), col++, row);
      gridPane.add(resetMaxProbabilityButton(), col++, row);

      gridPane.prefHeightProperty().bind(scene.heightProperty());
      gridPane.prefWidthProperty().bind(scene.widthProperty());

      root.getChildren().add(gridPane);
      advanced.setScene(scene);

      return advanced;
   }

   private final List<Stoppable> listeners = new ArrayList<>();

   public void stop()
   {
      for (Stoppable listener : listeners)
         listener.stop();
   }

   private static interface Stoppable
   {
      public void stop();
   }
}
