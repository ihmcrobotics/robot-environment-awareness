package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.io.File;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;

public class DataExporterAnchorPaneController extends REABasicUIController
{
   @FXML
   private TextField currentDataFolderTextField;

   private final DirectoryChooser directoryChooser = new DirectoryChooser();
   private Window ownerWindow;

   public DataExporterAnchorPaneController()
   {
   }

   public void setMainWindow(Window ownerWindow)
   {
      this.ownerWindow = ownerWindow;
   }

   @Override
   public void bindControls()
   {
      File defaultDataFile = new File("Data");
      directoryChooser.setInitialDirectory(defaultDataFile);
      currentDataFolderTextField.setText(defaultDataFile.getAbsolutePath());
   }

   @FXML
   private void browseOutputFolder()
   {
      String newPath = directoryChooser.showDialog(ownerWindow).getAbsolutePath();
      uiMessager.submitMessageInternal(REAModuleAPI.UIDataExporterDirectory, newPath);
      Platform.runLater(() -> currentDataFolderTextField.setText(newPath));
   }

   @FXML
   private void exportSegmentation()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UIPlanarRegionExportSegmentation, true);
   }
}
