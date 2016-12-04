package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.io.File;

import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.ToggleButton;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;

public abstract class REABasicUIController
{
   protected REAUIMessager uiMessager;
   private FilePropertyHelper filePropertyHelper;

   public abstract void bindControls();

   public REABasicUIController()
   {
   }

   public void attachREAMessager(REAUIMessager uiMessager)
   {
      this.uiMessager = uiMessager;
   }

   public void setConfigurationFile(File configurationFile)
   {
      filePropertyHelper = new FilePropertyHelper(configurationFile);
   }

   protected void saveUIControlProperty(String propertyName, ToggleButton toggleButton)
   {
      filePropertyHelper.saveProperty(propertyName, toggleButton.isSelected());
   }

   protected void saveUIControlProperty(String propertyName, ComboBox<?> comboBox)
   {
      filePropertyHelper.saveProperty(propertyName, comboBox.getValue().toString());
   }

   protected void saveUIControlProperty(String propertyName, Slider slider)
   {
      filePropertyHelper.saveProperty(propertyName, slider.getValue());
   }

   protected void loadUIControlProperty(String propertyName, ToggleButton toggleButton)
   {
      Boolean loadedProperty = filePropertyHelper.loadBooleanProperty(propertyName);
      if (loadedProperty != null)
         toggleButton.setSelected(loadedProperty);
   }

   protected void loadUIControlProperty(String propertyName, Slider slider)
   {
      Double loadedProperty = filePropertyHelper.loadDoubleProperty(propertyName);
      if (loadedProperty != null)
         slider.setValue(loadedProperty);
   }

   protected void loadUIControlProperty(String propertyName, Spinner<Double> spinner)
   {
      Double loadedProperty = filePropertyHelper.loadDoubleProperty(propertyName);
      if (loadedProperty != null)
         spinner.getValueFactory().setValue(loadedProperty);
   }

   @SuppressWarnings("unchecked")
   protected <T extends Enum<T>> void loadUIControlProperty(String propertyName, ComboBox<T> comboBox)
   {
      String loadedProperty = filePropertyHelper.loadProperty(propertyName);
      if (loadedProperty != null)
      {
         T valueOf = (T) Enum.valueOf(comboBox.getItems().get(0).getClass(), loadedProperty);
         comboBox.setValue(valueOf);
      }
   }
}