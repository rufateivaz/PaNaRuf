#ifndef  PANORAMA_TYPE_H  
#define  PANORAMA_TYPE_H 

/*
	Since, our software can create spherical, cylindrical and perspective warping,
	we use the below enum "PanoramaType" to categorize them based on the chosen input.
*/
enum PanoramaType { PERSPECTIVE, CYLINDRICAL, SPHERICAL , NONE };

#endif 