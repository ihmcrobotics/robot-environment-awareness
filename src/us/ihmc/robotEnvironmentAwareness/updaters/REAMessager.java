package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.List;

import us.ihmc.robotEnvironmentAwareness.communication.REAMessage;

public interface REAMessager
{

   void submitMessage(REAMessage message);

   List<REAMessage> getUnprocessedMessages();

}