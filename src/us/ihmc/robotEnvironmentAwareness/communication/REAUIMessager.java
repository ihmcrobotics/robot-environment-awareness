package us.ihmc.robotEnvironmentAwareness.communication;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import javafx.beans.property.Property;
import javafx.beans.value.ObservableValue;
import us.ihmc.communication.net.NetStateListener;

public class REAUIMessager
{
   private final REAMessagerSharedVariables internalMessager = new REAMessagerSharedVariables();
   private final REAMessager reaMessagerToModule;

   public REAUIMessager(REAMessager reaMessagerToModule)
   {
      this.reaMessagerToModule = reaMessagerToModule;
   }

   public <T> AtomicReference<T> createInput(String topic)
   {
      return createInput(topic, null);
   }

   public <T> AtomicReference<T> createInput(String topic, T defaultValue)
   {
      AtomicReference<T> input = internalMessager.createInput(topic, defaultValue);
      reaMessagerToModule.registerTopicListener(topic, input::set);
      return input;
   }

   public void broadcastMessage(String topic, Object messageContent)
   {
      submitMessageToModule(topic, messageContent);
      submitMessageInternal(topic, messageContent);
   }

   public void broadcastMessage(REAMessage message)
   {
      submitMessageToModule(message);
      submitMessageInternal(message);
   }

   public void submitStateRequestToModule(String requestTopic)
   {
      reaMessagerToModule.submitStateRequest(requestTopic);
   }

   public void submitMessageToModule(String topic, Object messageContent)
   {
      reaMessagerToModule.submitMessage(topic, messageContent);
   }

   public void submitMessageToModule(REAMessage message)
   {
      reaMessagerToModule.submitMessage(message);
   }

   public void submitMessageInternal(String topic, Object messageContent)
   {
      internalMessager.submitMessage(topic, messageContent);
   }

   public void submitMessageInternal(REAMessage message)
   {
      internalMessager.submitMessage(message);
   }

   public <T> void registerTopicListener(String topic, REATopicListener<T> listener)
   {
      internalMessager.registerTopicListener(topic, listener);
      reaMessagerToModule.registerTopicListener(topic, listener);
   }

   public <T> void bindBidirectionalInternal(String topic, Property<T> property)
   {
      MessageBidirectionalBinding<T> bind = new MessageBidirectionalBinding<>(property, messageContent -> submitMessageInternal(topic, messageContent));
      property.addListener(bind);
      internalMessager.registerTopicListener(topic, bind);
   }

   public <T> void bindBidirectionalModule(String topic, Property<T> property)
   {
      MessageBidirectionalBinding<T> bind = new MessageBidirectionalBinding<>(property, messageContent -> submitMessageToModule(topic, messageContent));
      property.addListener(bind);
      reaMessagerToModule.registerTopicListener(topic, bind);
   }

   public <T> void bindBidirectionalGlobal(String topic, Property<T> property)
   {
      MessageBidirectionalBinding<T> bind = new MessageBidirectionalBinding<>(property, messageContent -> broadcastMessage(topic, messageContent));
      property.addListener(bind);
      internalMessager.registerTopicListener(topic, bind);
      reaMessagerToModule.registerTopicListener(topic, bind);
   }

   public <T> void bindPropertyToTopic(String topic, Property<T> propertyToBind)
   {
      reaMessagerToModule.registerTopicListener(topic, propertyToBind::setValue);
      internalMessager.registerTopicListener(topic, propertyToBind::setValue);
   }

   public <T> void bindInternalTopic(String topic, ObservableValue<T> observableValue)
   {
      observableValue.addListener((observable) -> submitMessageInternal(topic, observableValue.getValue()));
   }

   public <T> void bindModuleTopic(String topic, ObservableValue<T> observableValue)
   {
      observableValue.addListener((observable) -> submitMessageToModule(topic, observableValue.getValue()));
   }
   public <T> void bindGlobal(String topic, ObservableValue<T> observableValue)
   {
      observableValue.addListener((observable) -> broadcastMessage(topic, observableValue.getValue()));
   }

   public void startMessager() throws IOException
   {
      internalMessager.startMessager();
      reaMessagerToModule.startMessager();
   }

   public void closeMessager()
   {
      internalMessager.closeMessager();
      reaMessagerToModule.closeMessager();
   }

   public void registerModuleConnectionStateListener(NetStateListener listener)
   {
      reaMessagerToModule.registerConnectionStateListener(listener);
   }
}
