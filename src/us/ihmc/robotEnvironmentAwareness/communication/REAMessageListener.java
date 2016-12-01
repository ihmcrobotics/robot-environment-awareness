package us.ihmc.robotEnvironmentAwareness.communication;

public interface REAMessageListener<T>
{
   public void receivedREAMessage(T messageContent);
}
