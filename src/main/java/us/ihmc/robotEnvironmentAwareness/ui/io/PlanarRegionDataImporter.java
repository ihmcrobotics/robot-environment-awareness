package us.ihmc.robotEnvironmentAwareness.ui.io;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.communication.packets.PlanarRegionMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionDataImporter
{
   private final File dataFolder;
   private PlanarRegionsListMessage planarRegionData = new PlanarRegionsListMessage();

   public static PlanarRegionDataImporter createImporterWithFileChooser(Window ownerWindow)
   {
      DirectoryChooser directoryChooser = new DirectoryChooser();
      File initialDirectory = new File("../../Data/PlanarRegion");
      if (!initialDirectory.exists() || !initialDirectory.isDirectory())
         initialDirectory = new File(".");
      directoryChooser.setInitialDirectory(initialDirectory);

      File result = directoryChooser.showDialog(ownerWindow);
      if (result == null)
         return null;
      else
         return new PlanarRegionDataImporter(result);
   }

   public PlanarRegionDataImporter(File dataFolder)
   {
      this.dataFolder = dataFolder;
   }

   public void loadPlanarRegionData() throws IOException
   {
      planarRegionData = new PlanarRegionsListMessage();
      planarRegionData.planarRegions = new ArrayList<>();

      File headerFile = new File(dataFolder, "header.txt");
      Map<PlanarRegionMessage, String> regionIdToFilename = loadHeader(headerFile);
      loadAllRegions(regionIdToFilename);
   }

   private Map<PlanarRegionMessage, String> loadHeader(File headerFile) throws IOException
   {
      Map<PlanarRegionMessage, String> regionIdToFilename = new HashMap<>();

      FileReader fileReader = new FileReader(headerFile);
      BufferedReader bufferedReader = new BufferedReader(fileReader);
      String line = "";
      String cvsSplitBy = ",";

      while ((line = bufferedReader.readLine()) != null)
      {
         line = line.replaceAll("regionId: ", "");
         line = line.replaceAll("index: ", "");
         line = line.replaceAll("origin: ", "");
         line = line.replaceAll("normal: ", "");
         line = line.replaceAll("\\[", "");
         line = line.replaceAll("\\]", "");
         line = line.replaceAll("concave hull size: ", "");
         line = line.replaceAll("number of convex polygons: ", "");
         line = line.replaceAll(" ", "");
         String[] values = line.split(cvsSplitBy);

         int i = 0;
         PlanarRegionMessage data = new PlanarRegionMessage();
         data.regionId = Integer.parseInt(values[i++]);
         int regionIndex = Integer.parseInt(values[i++]);
         regionIdToFilename.put(data, "region" + data.regionId + "_" + regionIndex);

         float xOrigin = Float.parseFloat(values[i++]);
         float yOrigin = Float.parseFloat(values[i++]);
         float zOrigin = Float.parseFloat(values[i++]);
         data.regionOrigin = new Point3D32(xOrigin, yOrigin, zOrigin);

         float xNormal = Float.parseFloat(values[i++]);
         float yNormal = Float.parseFloat(values[i++]);
         float zNormal = Float.parseFloat(values[i++]);
         data.regionNormal = new Vector3D32(xNormal, yNormal, zNormal);

         data.concaveHullVertices = new Point2D32[Integer.parseInt(values[i++])];

         int nConvexPolygons = Integer.parseInt(values[i++]);
         data.convexPolygonsVertices = new ArrayList<>();

         for (int hullIndex = 0; hullIndex < nConvexPolygons; hullIndex++)
            data.convexPolygonsVertices.add(new Point2D32[Integer.parseInt(values[i++])]);

         planarRegionData.planarRegions.add(data);
      }

      bufferedReader.close();

      return regionIdToFilename;
   }

   private void loadAllRegions(Map<PlanarRegionMessage, String> regionIdToFilename)
   {
      planarRegionData.planarRegions.parallelStream().forEach(region -> loadRegion(region, regionIdToFilename));
   }

   private void loadRegion(PlanarRegionMessage regionToLoad, Map<PlanarRegionMessage, String> regionIdToFilename)
   {
      try
      {
         String fileName = regionIdToFilename.remove(regionToLoad);
         File regionFile = new File(dataFolder, fileName);
         FileReader fileReader = new FileReader(regionFile);
         BufferedReader bufferedReader = new BufferedReader(fileReader);

         String line = "";
         String cvsSplitBy = ",";

         List<Point2D32> loadedPoints = new ArrayList<>();

         while ((line = bufferedReader.readLine()) != null)
         {
            String[] coordsAsString = line.split(cvsSplitBy);
            float x = Float.parseFloat(coordsAsString[0]);
            float y = Float.parseFloat(coordsAsString[1]);
            loadedPoints.add(new Point2D32(x, y));
         }

         bufferedReader.close();

         for (int i = 0; i < regionToLoad.concaveHullVertices.length; i++)
         {
            regionToLoad.concaveHullVertices[i] = loadedPoints.remove(0);
         }

         for (int polygonIndex = 0; polygonIndex < regionToLoad.convexPolygonsVertices.size(); polygonIndex++)
         {
            Point2D32[] currentArrayToFill = regionToLoad.convexPolygonsVertices.get(polygonIndex);
            for (int i = 0; i < currentArrayToFill.length; i++)
               currentArrayToFill[i] = loadedPoints.remove(0);
         }
      }
      catch (IOException e)
      {
         return;
      }
   }

   public PlanarRegionsListMessage getPlanarRegionData()
   {
      return planarRegionData;
   }

   public PlanarRegionsList getAsPlanarRegionsList()
   {
      return PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionData);
   }
}
