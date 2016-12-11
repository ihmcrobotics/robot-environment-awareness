package us.ihmc.robotEnvironmentAwareness.ui.io;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;

public class PlanarRegionSegmentationDataImporter
{
   private final File dataFolder;
   private final List<PlanarRegionSegmentationMessage> planarRegionSegmentationData = new ArrayList<>();

   public static PlanarRegionSegmentationDataImporter createImporterWithFileChooser(Window ownerWindow)
   {
      DirectoryChooser directoryChooser = new DirectoryChooser();
      directoryChooser.setInitialDirectory(new File("Data"));
      File result = directoryChooser.showDialog(ownerWindow);
      return new PlanarRegionSegmentationDataImporter(result);
   }

   public PlanarRegionSegmentationDataImporter(File dataFolder)
   {
      this.dataFolder = dataFolder;
   }

   public void loadPlanarRegionSegmentationData() throws IOException
   {
      File headerFile = new File(dataFolder, "header.txt");
      loadHeader(headerFile);
      loadAllRegions();
   }

   private void loadHeader(File headerFile) throws IOException
   {
      FileReader fileReader = new FileReader(headerFile);
      BufferedReader bufferedReader = new BufferedReader(fileReader);
      String line = "";
      String cvsSplitBy = ",";

      while ((line = bufferedReader.readLine()) != null)
      {
         line = line.replaceAll("regionId: ", "");
         line = line.replaceAll("origin: ", "");
         line = line.replaceAll("normal: ", "");
         String[] values = line.split(cvsSplitBy);
         
         int regionId = Integer.parseInt(values[0]);
         float xOrigin = Float.parseFloat(values[1]);
         float yOrigin = Float.parseFloat(values[2]);
         float zOrigin = Float.parseFloat(values[3]);
         float xNormal = Float.parseFloat(values[4]);
         float yNormal = Float.parseFloat(values[5]);
         float zNormal = Float.parseFloat(values[6]);

         PlanarRegionSegmentationMessage planarRegionSegmentation = new PlanarRegionSegmentationMessage();
         planarRegionSegmentation.id = regionId;
         planarRegionSegmentation.origin = new Point3f(xOrigin, yOrigin, zOrigin);
         planarRegionSegmentation.normal = new Vector3f(xNormal, yNormal, zNormal);
         planarRegionSegmentationData.add(planarRegionSegmentation);
      }

      bufferedReader.close();
   }

   private void loadAllRegions()
   {
      planarRegionSegmentationData.parallelStream().forEach(this::loadRegion);
   }

   private void loadRegion(PlanarRegionSegmentationMessage regionToLoad)
   {
      try
      {
         File regionFile = new File(dataFolder, "region" + Integer.toString(regionToLoad.getRegionId()));
         FileReader fileReader = new FileReader(regionFile);
         BufferedReader bufferedReader = new BufferedReader(fileReader);

         String line = "";
         String cvsSplitBy = ",";

         List<Point3f> loadedPoints = new ArrayList<>();
         
         while ((line = bufferedReader.readLine()) != null)
         {
            String[] coordsAsString = line.split(cvsSplitBy);
            float x = Float.parseFloat(coordsAsString[0]);
            float y = Float.parseFloat(coordsAsString[1]);
            float z = Float.parseFloat(coordsAsString[2]);
            loadedPoints.add(new Point3f(x, y, z));
         }
         regionToLoad.hitLocations = loadedPoints.toArray(new Point3f[0]);

         bufferedReader.close();
      }
      catch (IOException e)
      {
      }
   }

   public List<PlanarRegionSegmentationMessage> getPlanarRegionSegmentationData()
   {
      return planarRegionSegmentationData;
   }
}
