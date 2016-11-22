//package us.ihmc.robotEnvironmentAwareness.ui;
//
//import javafx.application.Application;
//import javafx.application.Platform;
//import javafx.fxml.FXMLLoader;
//import javafx.scene.Scene;
//import javafx.scene.layout.BorderPane;
//import javafx.stage.Stage;
//import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAService;
//
//import java.io.IOException;
//
///**
// * Created by adrien on 11/16/16.
// */
//public class LIDARBasedREAModuleLauncher extends Application {
//
//    private final LIDARBasedREAService lidarBasedREAService;
//
//    private final BorderPane mainPane;
//
//    public LIDARBasedREAModuleLauncher() throws IOException {
//
//        lidarBasedREAService = new LIDARBasedREAService();
//        FXMLLoader loader = new FXMLLoader();
//        loader.setController(this);
//        loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
//        mainPane = loader.load();
//
//    }
//
//    @Override
//    public void start(Stage primaryStage) throws Exception
//    {
//        lidarBasedREAService.start(); // TODO add button to start and stop service instead
//        primaryStage.setTitle(getClass().getSimpleName());
//        primaryStage.setMaximized(true);
//        Scene mainScene = new Scene(mainPane, 600, 400);
//        primaryStage.setScene(mainScene);
//        primaryStage.show();
//        primaryStage.setOnCloseRequest(event -> stop());
//    }
//
//    @Override
//    public void stop()
//    {
//        try
//        {
//            lidarBasedREAService.stop();
//            Platform.exit();
//        }
//        catch (Exception e)
//        {
//            e.printStackTrace();
//        }
//    }
//
//    public static void main(String[] args)
//    {
//        launch(args);
//    }
//
//}
