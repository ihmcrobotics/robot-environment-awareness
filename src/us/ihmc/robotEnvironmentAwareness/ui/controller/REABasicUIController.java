package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.HashSet;
import java.util.Properties;
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
   private File configurationFile;
   private final Set<InvalidationListener> invalidationListeners = new HashSet<>();

   public abstract void bindControls();

   public REABasicUIController()
   {
   }

   public void setConfigurationFile(File configurationFile)
   {
      this.configurationFile = configurationFile;
   }

   protected void saveProperty(String propertyName, double propertyValue)
   {
      saveProperty(propertyName, Double.toString(propertyValue));
   }

   protected void saveProperty(String propertyName, int propertyValue)
   {
      saveProperty(propertyName, Integer.toString(propertyValue));
   }

   protected void saveProperty(String propertyName, boolean propertyValue)
   {
      saveProperty(propertyName, Boolean.toString(propertyValue));
   }

   protected void saveProperty(String propertyName, String propertyValue)
   {
      FileOutputStream fileOut = null;
      FileInputStream fileIn = null;

      try
      {
         Properties properties = new Properties();

         if (configurationFile.exists() && configurationFile.isFile())
         {
            fileIn = new FileInputStream(configurationFile);
            properties.load(fileIn);
         }

         properties.setProperty(propertyName, propertyValue);
         fileOut = new FileOutputStream(configurationFile);
         properties.store(fileOut, "");
         fileOut.close();
      }
      catch (Exception ex)
      {
         throw new RuntimeException("Problem when saving property.");
      }
      finally
      {
         try
         {
            if (fileIn != null)
               fileIn.close();
         }
         catch (Exception e)
         {
         }

         try
         {
            fileOut.close();
         }
         catch (IOException e)
         {
         }
      }
   }

   protected void loadPropertyAndUpdateUIControl(ToggleButton toggleButton, String propertyName)
   {
      String loadedProperty = loadProperty(propertyName);
      if (loadedProperty != null)
         toggleButton.setSelected(Boolean.parseBoolean(loadedProperty));
   }

   protected void loadPropertyAndUpdateUIControl(Slider slider, String propertyName)
   {
      String loadedProperty = loadProperty(propertyName);
      if (loadedProperty != null)
         slider.setValue(Double.parseDouble(loadedProperty));
   }

   protected void loadPropertyAndUpdateUIControl(Spinner<Double> spinner, String propertyName)
   {
      String loadedProperty = loadProperty(propertyName);
      if (loadedProperty != null)
         spinner.getValueFactory().setValue(Double.parseDouble(loadedProperty));
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
      FileInputStream fileIn = null;
      String propertyValue = null;

      if (!configurationFile.exists() || !configurationFile.isFile())
         return null;

      try
      {
         Properties properties = new Properties();

         fileIn = new FileInputStream(configurationFile);
         properties.load(fileIn);
         propertyValue = properties.getProperty(propertyName);
      }
      catch (Exception ex)
      {
         throw new RuntimeException("Problem when loading property.");
      }
      finally
      {
         try
         {
            fileIn.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      return propertyValue;
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