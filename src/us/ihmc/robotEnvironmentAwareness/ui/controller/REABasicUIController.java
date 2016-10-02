package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.beans.property.Property;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessager;

public abstract class REABasicUIController
{

   private REAMessager outputMessager;

   public abstract void bindControls();

   public REABasicUIController()
   {
      super();
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
      ListenerTools.sendMessageOnPropertyChange(property, outputMessager, messageName);
   }

   protected void send(REAMessage message)
   {
      outputMessager.submitMessage(message);
   }
}