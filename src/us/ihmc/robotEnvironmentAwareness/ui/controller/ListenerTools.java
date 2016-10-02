package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.beans.InvalidationListener;
import javafx.beans.Observable;
import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;
import us.ihmc.robotEnvironmentAwareness.updaters.REAMessager;

public class ListenerTools
{
   public static <T> void sendMessageOnPropertyChange(Property<T> property, REAMessager messager, String messageName)
   {
      property.addListener(new InvalidationListener()
      {
         @Override
         public void invalidated(Observable observable)
         {
            messager.submitMessage(new REAMessage(messageName, property.getValue()));
         }
      });
   }
}
