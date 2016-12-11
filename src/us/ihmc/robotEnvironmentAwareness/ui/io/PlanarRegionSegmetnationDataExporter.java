package us.ihmc.robotEnvironmentAwareness.ui.io;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.concurrent.Executor;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;

public class PlanarRegionSegmetnationDataExporter
{
   private final Executor executor = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   private final AtomicReference<PlanarRegionSegmentationMessage[]> planarRegionSegmentationState;
   private final AtomicReference<String> dataDirectoryPath;

   public PlanarRegionSegmetnationDataExporter(REAUIMessager uiMessager)
   {
      planarRegionSegmentationState = uiMessager.createInput(REAModuleAPI.PlanarRegionsSegmentationState);
      dataDirectoryPath = uiMessager.createInput(REAModuleAPI.UIDataExporterDirectory, new File("Data/").getAbsolutePath());
      uiMessager.registerTopicListener(REAModuleAPI.UIPlanarRegionExportSegmentation, this::exportSegmentationData);
   }

   private void exportSegmentationData(boolean export)
   {
      PlanarRegionSegmentationMessage[] segmentationData = planarRegionSegmentationState.get();
      if (segmentationData != null)
         executor.execute(() -> executeOnThread(segmentationData));
   }

   private void executeOnThread(PlanarRegionSegmentationMessage[] segmentationData)
   {
      Path folderPath = Paths.get(dataDirectoryPath.get() + File.separator + getDate() + "PlanarRegionSegmentation");
      try
      {
         Files.createDirectories(folderPath);
         File header = new File(folderPath.toFile(), "header.txt");
         writeHeaderFile(header, segmentationData);
         writeRegionData(folderPath, segmentationData);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return;
      }
   }

   private void writeHeaderFile(File header, PlanarRegionSegmentationMessage[] segmentationData) throws IOException
   {
      FileWriter fileWriter = new FileWriter(header);

      for (PlanarRegionSegmentationMessage message : segmentationData)
      {
         Point3f origin = message.getOrigin();
         Vector3f normal = message.getNormal();
         fileWriter.write("regionId: ");
         fileWriter.write(Integer.toString(message.getRegionId()));
         fileWriter.write(", origin: ");
         fileWriter.write(origin.getX() + ", " + origin.getY() + ", " + origin.getZ());
         fileWriter.write(", normal: ");
         fileWriter.write(normal.getX() + ", " + normal.getY() + ", " + normal.getZ());
         fileWriter.write("\n");
      }

      fileWriter.close();
   }

   private void writeRegionData(Path folderPath, PlanarRegionSegmentationMessage[] segmentationData) throws IOException
   {
      for (PlanarRegionSegmentationMessage message : segmentationData)
      {
         File regionFile = new File(folderPath.toFile(), "region" + message.getRegionId());
         FileWriter fileWriter = new FileWriter(regionFile);

         for (Point3f hitLocation : message.getHitLocations())
         {
            fileWriter.write(hitLocation.getX() + ", " + hitLocation.getY() + ", " + hitLocation.getZ() + "\n");
         }

         fileWriter.close();
      }
   }

   private static String getDate()
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss_");
      Date time = Calendar.getInstance().getTime();
      return dateFormat.format(time);
   }
}
