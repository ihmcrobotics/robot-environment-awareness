package us.ihmc.robotEnvironmentAwareness.communication.converters;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.boundingBox.OcTreeSimpleBoundingBox;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoxMessage;

public class BoundingBoxMessageConverter
{

   public static BoxMessage convertToMessage(OcTreeBoundingBoxInterface boundingBox)
   {
      if (boundingBox == null)
         return BoxMessage.emptyBox();

      if (boundingBox instanceof OcTreeBoundingBoxWithCenterAndYaw)
         return convertToMessage((OcTreeBoundingBoxWithCenterAndYaw) boundingBox);
      else if (boundingBox instanceof OcTreeSimpleBoundingBox)
         return convertToMessage((OcTreeSimpleBoundingBox) boundingBox);
      else
         throw new RuntimeException("No conversion implemented for this bounding box: " + boundingBox.getClass().getSimpleName());
   }

   public static BoxMessage convertToMessage(OcTreeSimpleBoundingBox boundingBox)
   {
      Quat4d orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      Point3d center = new Point3d();
      Vector3d size = new Vector3d();
      boundingBox.getSize(size);
      boundingBox.getCenterCoordinate(center);

      BoxMessage boxMessage = new BoxMessage();
      boxMessage.setCenter(center);
      boxMessage.setOrientation(orientation);
      boxMessage.setSize(size);
      return boxMessage;
   }
   
   public static BoxMessage convertToMessage(OcTreeBoundingBoxWithCenterAndYaw boundingBox)
   {
      Point3d center = new Point3d();
      Vector3d size = new Vector3d();
      boundingBox.getLocalSize(size);
      boundingBox.getCenterCoordinate(center);
      Quat4d orientation = new Quat4d(0.0, 0.0, Math.sin(0.5 * boundingBox.getYaw()), Math.cos(0.5 * boundingBox.getYaw()));
      
      BoxMessage boxMessage = new BoxMessage();
      boxMessage.setCenter(center);
      boxMessage.setOrientation(orientation);
      boxMessage.setSize(size);
      return boxMessage;
   }
}
