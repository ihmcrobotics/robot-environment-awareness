package us.ihmc.robotEnvironmentAwareness;

import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;

public class RemoteLidarBasedREAUILauncher extends Application
{
   private static final String HOST = "localhost";

   public RemoteLidarBasedREAUILauncher()
   {
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      LIDARBasedEnvironmentAwarenessUI remoteUI = LIDARBasedEnvironmentAwarenessUI.creatRemoteUI(primaryStage, HOST);
      remoteUI.show();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
