package robotenvironmentawareness_msgs.msg.dds;
/**
* 
* Definition of the class "PolygonizerParameters" defined in PolygonizerParameters_.idl. 
*
* This file was automatically generated from PolygonizerParameters_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PolygonizerParameters_.idl instead.
*
*/
public class PolygonizerParameters
{
    public PolygonizerParameters()
    {
        
        
    }

    public void set(PolygonizerParameters other)
    {
        	concave_hull_threshold_ = other.concave_hull_threshold_;
        	min_number_of_nodes_ = other.min_number_of_nodes_;
        	shallow_angle_threshold_ = other.shallow_angle_threshold_;
        	peak_angle_threshold_ = other.peak_angle_threshold_;
        	length_threshold_ = other.length_threshold_;
        	depth_threshold_ = other.depth_threshold_;

    }

    public void setConcave_hull_threshold(double concave_hull_threshold)
    {
        concave_hull_threshold_ = concave_hull_threshold;
    }

    public double getConcave_hull_threshold()
    {
        return concave_hull_threshold_;
    }

        
    public void setMin_number_of_nodes(long min_number_of_nodes)
    {
        min_number_of_nodes_ = min_number_of_nodes;
    }

    public long getMin_number_of_nodes()
    {
        return min_number_of_nodes_;
    }

        
    public void setShallow_angle_threshold(double shallow_angle_threshold)
    {
        shallow_angle_threshold_ = shallow_angle_threshold;
    }

    public double getShallow_angle_threshold()
    {
        return shallow_angle_threshold_;
    }

        
    public void setPeak_angle_threshold(double peak_angle_threshold)
    {
        peak_angle_threshold_ = peak_angle_threshold;
    }

    public double getPeak_angle_threshold()
    {
        return peak_angle_threshold_;
    }

        
    public void setLength_threshold(double length_threshold)
    {
        length_threshold_ = length_threshold;
    }

    public double getLength_threshold()
    {
        return length_threshold_;
    }

        
    public void setDepth_threshold(double depth_threshold)
    {
        depth_threshold_ = depth_threshold;
    }

    public double getDepth_threshold()
    {
        return depth_threshold_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof PolygonizerParameters)) return false;
        PolygonizerParameters otherMyClass = (PolygonizerParameters)other;
        boolean returnedValue = true;

        returnedValue &= this.concave_hull_threshold_ == otherMyClass.concave_hull_threshold_;

                
        returnedValue &= this.min_number_of_nodes_ == otherMyClass.min_number_of_nodes_;

                
        returnedValue &= this.shallow_angle_threshold_ == otherMyClass.shallow_angle_threshold_;

                
        returnedValue &= this.peak_angle_threshold_ == otherMyClass.peak_angle_threshold_;

                
        returnedValue &= this.length_threshold_ == otherMyClass.length_threshold_;

                
        returnedValue &= this.depth_threshold_ == otherMyClass.depth_threshold_;

                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("PolygonizerParameters {");
        builder.append("concave_hull_threshold=");
        builder.append(this.concave_hull_threshold_);

                builder.append(", ");
        builder.append("min_number_of_nodes=");
        builder.append(this.min_number_of_nodes_);

                builder.append(", ");
        builder.append("shallow_angle_threshold=");
        builder.append(this.shallow_angle_threshold_);

                builder.append(", ");
        builder.append("peak_angle_threshold=");
        builder.append(this.peak_angle_threshold_);

                builder.append(", ");
        builder.append("length_threshold=");
        builder.append(this.length_threshold_);

                builder.append(", ");
        builder.append("depth_threshold=");
        builder.append(this.depth_threshold_);

                
        builder.append("}");
		return builder.toString();
    }

    private double concave_hull_threshold_; 
    private long min_number_of_nodes_; 
    private double shallow_angle_threshold_; 
    private double peak_angle_threshold_; 
    private double length_threshold_; 
    private double depth_threshold_; 

}