package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.io.File;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;

public class DataExporterAnchorPaneController extends REABasicUIController
{
   private final File defaultSegmentationDataFile = new File("Data/Segmentation/");
   private final File defaultPlanarRegionDataFile = new File("Data/PlanarRegion/");

   @FXML
   private TextField currentSegmentationDataFolderTextField;
   @FXML
   private TextField currentPlanarRegionDataFolderTextField;

   private final DirectoryChooser segmentationDirectoryChooser = new DirectoryChooser();
   private final DirectoryChooser planarRegionDirectoryChooser = new DirectoryChooser();
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
      currentSegmentationDataFolderTextField.setText(defaultSegmentationDataFile.getAbsolutePath());
      currentPlanarRegionDataFolderTextField.setText(defaultPlanarRegionDataFile.getAbsolutePath());
   }

   @FXML
   private void browseSegmentationOutputFolder()
   {
      segmentationDirectoryChooser.setInitialDirectory(defaultSegmentationDataFile);
      String newPath = segmentationDirectoryChooser.showDialog(ownerWindow).getAbsolutePath();
      uiMessager.submitMessageInternal(REAModuleAPI.UISegmentationDataExporterDirectory, newPath);
      Platform.runLater(() -> currentSegmentationDataFolderTextField.setText(newPath));
   }

   @FXML
   private void browsePlanarRegionOutputFolder()
   {
      planarRegionDirectoryChooser.setInitialDirectory(defaultPlanarRegionDataFile);
      String newPath = planarRegionDirectoryChooser.showDialog(ownerWindow).getAbsolutePath();
      uiMessager.submitMessageInternal(REAModuleAPI.UIPlanarRegionDataExporterDirectory, newPath);
      Platform.runLater(() -> currentPlanarRegionDataFolderTextField.setText(newPath));
   }

   @FXML
   private void exportSegmentation()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UISegmentationDataExportRequest, true);
   }

   @FXML
   private void exportPlanarRegion()
   {
      PrintTools.info("Request exporting planar region data");
      uiMessager.submitMessageInternal(REAModuleAPI.UIPlanarRegionDataExportRequest, true);
   }
}
