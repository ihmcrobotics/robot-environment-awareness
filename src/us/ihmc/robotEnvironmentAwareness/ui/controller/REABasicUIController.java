package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.io.File;
import java.util.HashSet;
import java.util.Set;

import javafx.beans.InvalidationListener;
import javafx.beans.property.Property;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.ToggleButton;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessager;

public abstract class REABasicUIController
{
   private REAMessager outputMessager;
   private FilePropertyHelper filePropertyHelper;
   private final Set<InvalidationListener> invalidationListeners = new HashSet<>();

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

   public void attachOutputMessager(REAMessager outputMessager)
   {
      this.outputMessager = outputMessager;
   }

   protected void sendMessageOnPropertyChange(ToggleButton toggleButton, String messageName)
   {
      sendMessageOnPropertyChange(toggleButton.selectedProperty(), messageName);
   }

   protected <T> void sendMessageOnPropertyChange(ComboBox<T> comboBox, String messageName)
   {
      sendMessageOnPropertyChange(comboBox.valueProperty(), messageName);
   }

   protected void sendMessageOnPropertyChange(Slider slider, String messageName)
   {
      sendMessageOnPropertyChange(slider.valueProperty(), messageName);
   }

   protected <T> void sendMessageOnPropertyChange(Property<T> property, String messageName)
   {
      invalidationListeners.add(ListenerTools.sendMessageOnPropertyChange(property, outputMessager, messageName));
   }

   protected void registerListener(InvalidationListener listener)
   {
      invalidationListeners.add(listener);
   }

   protected void fireAllListeners()
   {
      for (InvalidationListener invalidationListener : invalidationListeners)
         invalidationListener.invalidated(null);
   }

   protected void send(REAMessage message)
   {
      outputMessager.submitMessage(message);
   }
}