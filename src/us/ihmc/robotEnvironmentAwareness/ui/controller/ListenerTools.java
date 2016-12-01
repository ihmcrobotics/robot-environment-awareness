package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.beans.InvalidationListener;
import javafx.beans.Observable;
import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;

public class ListenerTools
{
   public static <T> InvalidationListener sendMessageOnPropertyChange(Property<T> property, REAMessager reaMessager, String topic)
   {
      InvalidationListener listener = new InvalidationListener()
      {
         @Override
         public void invalidated(Observable observable)
         {
            reaMessager.submitMessage(topic, property.getValue());
         }
      };
      property.addListener(listener);
      return listener;
   }
}
