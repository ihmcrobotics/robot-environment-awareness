package us.ihmc.robotEnvironmentAwareness;

import java.io.IOException;

import us.ihmc.robotEnvironmentAwareness.updaters.LidarBasedREAModule;

public class RemoteLidarBasedREAModuleLauncher
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   public static void main(String[] args) throws IOException
   {
      LidarBasedREAModule remoteModule = LidarBasedREAModule.createRemoteModule(MODULE_CONFIGURATION_FILE_NAME);
      remoteModule.start();
   }
}
