package us.ihmc.robotEnvironmentAwareness.ui;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.SplitPane;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.robotEnvironmentAwareness.communication.LidarSimulationNetClassList;

public class LIDARBasedEnvironmentAwarenessUI extends Application
{
   private final PacketCommunicator packetCommunicator;
   private FootstepPlannerUI3DPane ui3dpane;

   @FXML
   private Spinner<Double> boundingBoxMinXSpinner;

   public LIDARBasedEnvironmentAwarenessUI()
   {
      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient("localhost", NetworkPorts.BEHAVIOUR_MODULE_PORT,
            new LidarSimulationNetClassList());
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      
      SplitPane mainPane = loader.load();

      packetCommunicator.connect();
      ui3dpane = new FootstepPlannerUI3DPane(600, 400, packetCommunicator);
      mainPane.getItems().set(0, ui3dpane);

      DoubleSpinnerValueFactory value = new DoubleSpinnerValueFactory(-1, 1, 0.5, 0.1);
      boundingBoxMinXSpinner.setValueFactory(value);

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      primaryStage.setScene(new Scene(mainPane, 600, 400));
      primaryStage.show();

      primaryStage.setOnCloseRequest(new EventHandler<WindowEvent>()
      {
         @Override
         public void handle(WindowEvent event)
         {
            stop();
         }
      });
      
   }

   @Override
   public void stop()
   {
      try
      {
         packetCommunicator.closeConnection();
         packetCommunicator.close();
         if (ui3dpane != null)
            ui3dpane.stop();
         Platform.exit();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
