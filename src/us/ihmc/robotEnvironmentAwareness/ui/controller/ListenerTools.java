package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.beans.InvalidationListener;
import javafx.beans.Observable;
import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessager;

public class ListenerTools
{
   public static <T> InvalidationListener sendMessageOnPropertyChange(Property<T> property, REAMessager messager, String messageName)
   {
      InvalidationListener listener = new InvalidationListener()
      {
         @Override
         public void invalidated(Observable observable)
         {
            messager.submitMessage(new REAMessage(messageName, property.getValue()));
         }
      };
      property.addListener(listener);
      return listener;
   }
}
