package us.ihmc.robotEnvironmentAwareness.ui.obsolete;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.scene.control.Button;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;

public class PointCloudPropertyControlFactory
{
   private final PointCloudViewer pointCloudViewer;

   public PointCloudPropertyControlFactory(PointCloudViewer pointCloudViewer)
   {
      this.pointCloudViewer = pointCloudViewer;
   }

   public Slider pointCloudSizeSlider()
   {
      final IntegerProperty pointCloudSizeProperty = pointCloudViewer.pointCloudSizeProperty();
      Slider pointCloudSizeSlider = new Slider(0, 200000, pointCloudSizeProperty.intValue());
      pointCloudSizeSlider.setShowTickLabels(true);
      pointCloudSizeSlider.setShowTickMarks(true);
      pointCloudSizeSlider.setMajorTickUnit(50000);
      pointCloudSizeSlider.setBlockIncrement(5000);
      pointCloudSizeSlider.setPrefWidth(200);

      pointCloudSizeSlider.valueProperty().addListener(new ChangeListener<Number>()
      {
         @Override
         public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue)
         {
            pointCloudSizeProperty.set(newValue.intValue());
         }
      });

      return pointCloudSizeSlider;
   }

   public Button clearPointCloudButton()
   {
      final BooleanProperty pointCloudClearProperty = pointCloudViewer.clearPointCloudProperty();
      Button clearPointCloudButton = new Button("Clear point cloud");
      clearPointCloudButton.setOnAction(new EventHandler<ActionEvent>()
      {
         @Override
         public void handle(ActionEvent event)
         {
            pointCloudClearProperty.set(true);
         }
      });

      return clearPointCloudButton;
   }

   public ToggleButton enablePointCloudViewerButton()
   {
      final BooleanProperty enableProperty = pointCloudViewer.enableViewerProperty();
      final ToggleButton enableViewerButton = new ToggleButton("Enable point cloud viewer");
      enableViewerButton.setSelected(enableProperty.get());
      enableViewerButton.setOnAction(new EventHandler<ActionEvent>()
      {
         @Override
         public void handle(ActionEvent event)
         {
            enableProperty.set(enableViewerButton.isSelected());
         }
      });

      return enableViewerButton;
   }

   public void stop()
   {
   }
}
