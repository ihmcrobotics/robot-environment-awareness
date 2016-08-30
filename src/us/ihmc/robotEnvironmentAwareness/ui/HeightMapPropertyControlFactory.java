package us.ihmc.robotEnvironmentAwareness.ui;

import javafx.beans.property.BooleanProperty;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.scene.control.Button;
import javafx.scene.control.ToggleButton;

public class HeightMapPropertyControlFactory
{
   private final HeightMapViewer heightMapViewer;

   public HeightMapPropertyControlFactory(HeightMapViewer heightMapViewer)
   {
      this.heightMapViewer = heightMapViewer;
   }

   public Button clearHeightMapButton()
   {
      final BooleanProperty clearProperty = heightMapViewer.clearHeightMapProperty();
      Button clearPointCloudButton = new Button("Clear height map");
      clearPointCloudButton.setOnAction(new EventHandler<ActionEvent>()
      {
         @Override
         public void handle(ActionEvent event)
         {
            clearProperty.set(true);
         }
      });

      return clearPointCloudButton;
   }

   public ToggleButton enableHeightMapViewerButton()
   {
      final BooleanProperty enableProperty = heightMapViewer.enableProperty();
      final ToggleButton enableViewerButton = new ToggleButton("Enable height map viewer");
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
