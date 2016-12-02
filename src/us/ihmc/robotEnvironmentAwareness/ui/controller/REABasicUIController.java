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

   public void setConfigurationFile(File configurationFile)
   {
      filePropertyHelper = new FilePropertyHelper(configurationFile);
   }

   protected void saveProperty(String propertyName, double propertyValue)
   {
      filePropertyHelper.saveProperty(propertyName, propertyValue);
   }

   protected void saveProperty(String propertyName, int propertyValue)
   {
      filePropertyHelper.saveProperty(propertyName, propertyValue);
   }

   protected void saveProperty(String propertyName, boolean propertyValue)
   {
      filePropertyHelper.saveProperty(propertyName, propertyValue);
   }

   protected void saveProperty(String propertyName, String propertyValue)
   {
      filePropertyHelper.saveProperty(propertyName, propertyValue);
   }

   protected void loadPropertyAndUpdateUIControl(ToggleButton toggleButton, String propertyName)
   {
      Boolean loadedProperty = filePropertyHelper.loadBooleanProperty(propertyName);
      if (loadedProperty != null)
         toggleButton.setSelected(loadedProperty);
   }

   protected void loadPropertyAndUpdateUIControl(Slider slider, String propertyName)
   {
      Double loadedProperty = filePropertyHelper.loadDoubleProperty(propertyName);
      if (loadedProperty != null)
         slider.setValue(loadedProperty);
   }

   protected void loadPropertyAndUpdateUIControl(Spinner<Double> spinner, String propertyName)
   {
      Double loadedProperty = filePropertyHelper.loadDoubleProperty(propertyName);
      if (loadedProperty != null)
         spinner.getValueFactory().setValue(loadedProperty);
   }

   @SuppressWarnings("unchecked")
   protected <T extends Enum<T>> void loadPropertyAndUpdateUIControl(ComboBox<T> comboBox, String propertyName)
   {
      String loadedProperty = loadProperty(propertyName);
      if (loadedProperty != null)
      {
         T valueOf = (T) Enum.valueOf(comboBox.getItems().get(0).getClass(), loadedProperty);
         comboBox.setValue(valueOf);
      }
   }

   protected String loadProperty(String propertyName)
   {
      return filePropertyHelper.loadProperty(propertyName);
   }


   public void attachREAMessager(REAUIMessager uiMessager)
   {
      this.uiMessager = uiMessager;
   }
}