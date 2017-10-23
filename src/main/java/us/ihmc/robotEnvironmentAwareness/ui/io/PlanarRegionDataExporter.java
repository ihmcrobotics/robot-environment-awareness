package us.ihmc.robotEnvironmentAwareness.ui.io;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PlanarRegionMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionDataExporter
{
   private final Executor executor = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   private final AtomicReference<PlanarRegionsListMessage> planarRegionsState;
   private final AtomicReference<String> dataDirectoryPath;

   public PlanarRegionDataExporter(REAUIMessager uiMessager)
   {
      planarRegionsState = uiMessager.createInput(REAModuleAPI.PlanarRegionsState);
      dataDirectoryPath = uiMessager.createInput(REAModuleAPI.UIPlanarRegionDataExporterDirectory, new File("Data/PlanarRegions/").getAbsolutePath());
      uiMessager.registerTopicListener(REAModuleAPI.UIPlanarRegionDataExportRequest, this::exportPlanarRegionData);
   }

   public PlanarRegionDataExporter(File dataDirectoryPath)
   {
      planarRegionsState = new AtomicReference<>(null);
      this.dataDirectoryPath = new AtomicReference<>(dataDirectoryPath.getAbsolutePath());
   }

   public void exportPlanarRegionData(PlanarRegionsList planarRegionsList)
   {
      planarRegionsState.set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
      exportPlanarRegionData(true);
   }

   public void exportPlanarRegionData(PlanarRegion planarRegion)
   {
      planarRegionsState.set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(new PlanarRegionsList(planarRegion)));
      exportPlanarRegionData(true);
   }

   private void exportPlanarRegionData(boolean export)
   {
      PrintTools.info("Processing request exporting planar region data");
      PlanarRegionsListMessage planarRegionData = planarRegionsState.get();
      if (planarRegionData != null)
         executor.execute(() -> executeOnThread(planarRegionData));
   }

   private void executeOnThread(PlanarRegionsListMessage planarRegionData)
   {
      Path folderPath = Paths.get(dataDirectoryPath.get() + File.separator + getDate() + "PlanarRegion");
      try
      {
         if (folderPath.toFile().exists())
            return;
         Files.createDirectories(folderPath);
         File header = new File(folderPath.toFile(), "header.txt");
         writeHeaderFile(header, planarRegionData);
         writePlanarRegionData(folderPath, planarRegionData);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return;
      }
   }

   private void writeHeaderFile(File header, PlanarRegionsListMessage planarRegionData) throws IOException
   {
      FileWriter fileWriter = new FileWriter(header);

      for (PlanarRegionMessage message : planarRegionData.getPlanarRegions())
      {
         Point3D32 origin = message.getRegionOrigin();
         Vector3D32 normal = message.getRegionNormal();
         fileWriter.write("regionId: ");
         fileWriter.write(Integer.toString(message.getRegionId()));
         fileWriter.write(", origin: ");
         fileWriter.write(origin.getX() + ", " + origin.getY() + ", " + origin.getZ());
         fileWriter.write(", normal: ");
         fileWriter.write(normal.getX() + ", " + normal.getY() + ", " + normal.getZ());
         fileWriter.write(", number of concave hulls: ");

         List<Point2D32[]> concaveHulls = message.getConcaveHullsVertices();
         int[] concaveHullsSizes = new int[concaveHulls.size()];
         for (int i = 0; i < concaveHulls.size(); i++)
            concaveHullsSizes[i] = concaveHulls.get(i).length;

         fileWriter.write(concaveHulls.size() + " " + Arrays.toString(concaveHullsSizes));

         fileWriter.write(", number of convex polygons: ");

         List<Point2D32[]> convexPolygons = message.getConcaveHullsVertices();
         int[] convexPolygonsSizes = new int[convexPolygons.size()];
         for (int i = 0; i < convexPolygons.size(); i++)
            convexPolygonsSizes[i] = convexPolygons.get(i).length;

         fileWriter.write(convexPolygons.size() + " " + Arrays.toString(convexPolygonsSizes));

         fileWriter.write("\n");
      }

      fileWriter.close();
   }

   private void writePlanarRegionData(Path folderPath, PlanarRegionsListMessage planarRegionData) throws IOException
   {
      for (PlanarRegionMessage message : planarRegionData.getPlanarRegions())
      {
         File regionFile = new File(folderPath.toFile(), "region" + message.getRegionId());
         FileWriter fileWriter = new FileWriter(regionFile);

         List<Point2D32[]> concaveHullsVertices = message.getConcaveHullsVertices();
         for (Point2D32[] concaveHullVertices : concaveHullsVertices)
         {
            for (Point2D32 vertex : concaveHullVertices)
            {
               fileWriter.write(vertex.getX() + ", " + vertex.getY() + "\n");
            }
         }

         List<Point2D32[]> convexPolygonsVertices = message.getConcaveHullsVertices();
         for (Point2D32[] convexPolygonVertices : convexPolygonsVertices)
         {
            for (Point2D32 vertex : convexPolygonVertices)
            {
               fileWriter.write(vertex.getX() + ", " + vertex.getY() + "\n");
            }
         }

         fileWriter.close();
      }
   }

   private static String getDate()
   {
      DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_");
      return LocalDateTime.now().format(dateTimeFormatter);
   }
}
