#include "ros/ros.h"

#include <sensor_msgs/Image.h>

class TempratureVisualization

{

    ros ::Publisher pub_ ; 

    ros ::NodeHandle nh_ ; 

  public:
    
    TempratureVisualization ( ) ; 
    
    void ThermalCallback ( const sensor_msgs ::Image & msg ) ; 
    
    void ConvertImage ( const sensor_msgs ::Image & msg , 
                        sensor_msgs ::Image & image , 
                        int lowTemp , 
                        int highTemp ) ; 
    
} ; 

TempratureVisualization ::TempratureVisualization ( ) { 

  
  
}

TempratureVisualization ::ThermalCallback ( const sensor_msgs ::Image & msg ) { 

  const int minTemp = - 20 ; 
  const int maxTemp = 80 ; 

  sensor_msgs ::Image image ; 
  
  image .header .stamp = ros:: Time:: now ( ) ; 
  image .header .frame_id = msg .header .frame_id ; 
  image .height = msg .height ; 
  image .width = msg .width ; 
  image .step = msg .step ; 
  image .encoding = "rgb8" ; 
  
  TempratureVisualization ::ConvertImage ( msg , image , minTemp , maxTemp ) ; 
  
  this ->pub_ .publish ( image ) ; 
  
}

TempratureVisualization ::ConvertImage ( const sensor_msgs ::Image & msg , 
                                         sensor_msgs ::Image & image , 
                                         int lowTemp , 
                                         int highTemp ) { 
  
  for ( unsigned int i = 0 ; i < msg .width ; i++ ) { 

    for ( unsigned int j = 0 ; j < msg .height ; j++ ) { 
      
      float temp = msg .data [ i * msg .height + j ] ; 
      
      temp =  std ::min ( std ::max ( temp , lowTemp ) , highTemp ) ; 
      
      float red = ( temp - lowTemp ) * 255 / ( highTemp - lowTemp ) ; 
      
      float blue = 1 - red ; 
    
      image .data .push_back ( ( char ) red ) ; 
      image .data .push_back ( ( char ) 0.0 ) ; 
      image .data .push_back ( ( char ) blue ) ; 
    
    }
    
  }
  
}

int main ( int argc , char **argv )

{

  ros ::init ( argc , argv , "temprature_visualization" ) ; 

  TempratureVisualization temprature_viz ; 

  ros ::spin ( ) ; 

  return 0 ; 
  
}
