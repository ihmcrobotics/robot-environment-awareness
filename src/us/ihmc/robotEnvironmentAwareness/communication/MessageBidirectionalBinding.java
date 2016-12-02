package us.ihmc.robotEnvironmentAwareness.communication;

import java.util.concurrent.atomic.AtomicBoolean;

import javafx.beans.property.Property;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;

public class MessageBidirectionalBinding<T> implements REATopicListener<T>, ChangeListener<T>
{
   private final AtomicBoolean changedOnMessageReception = new AtomicBoolean(false);
   private final Property<T> boundProperty;
   private final MessagingAction<T> messagingAction;

   public MessageBidirectionalBinding(Property<T> boundProperty, MessagingAction<T> messagingAction)
   {
      this.boundProperty = boundProperty;
      this.messagingAction = messagingAction;
   }

   @Override
   public void changed(ObservableValue<? extends T> observable, T oldValue, T newValue)
   {
      if (changedOnMessageReception.getAndSet(false))
         return;
      messagingAction.doMessageAction(newValue);
   }

   @Override
   public void receivedMessageForTopic(T messageContent)
   {
      changedOnMessageReception.set(!messageContent.equals(boundProperty.getValue()));
      boundProperty.setValue(messageContent);
   }

   public interface MessagingAction<T>
   {
      public void doMessageAction(T messageContent);
   }
}
