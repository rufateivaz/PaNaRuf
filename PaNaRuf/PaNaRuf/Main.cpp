#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core.hpp>
#include <exception>
#ifdef HAVE_OPENCV_XFEATURES2D

// Header files
#include "IO.h"
#include "CustomCylindricalPanorama.h"
#include "CustomPerspectiveWarping.h"
#include "CustomSphericalPanorama.h"



using namespace std; 
using namespace cv; 
using namespace detail; 
using namespace xfeatures2d; 
using namespace samples;




int main(int argc, char* argv[])
{

	// Read the image names from the argv[1]
	vector<string> image_names;
	IO input_output;
	input_output.readImageNames(image_names, argv);		

	
	// Reading input arguments given by the user.


	PanoramaType panoType = NONE;
	if (argc >= 3) {
		String input2 = argv[2];
		if (input2.compare("-p")==0)
			panoType = PERSPECTIVE;
		else if (input2.compare("-c") == 0)
			panoType = CYLINDRICAL;
		else if (input2.compare("-s") == 0)
			panoType = SPHERICAL;
		else 
			input_output.provideCorrectInputError();	
	}

	else {
		input_output.provideCorrectInputError();
	}


	
	if (panoType == PERSPECTIVE) {
			CustomPerspectiveWarping perspectiveWarping;
			perspectiveWarping.applyCustomPerspectiveWarping(image_names);
	}
	else if (panoType == CYLINDRICAL) {
		CustomCylindricalPanorama cylindricalPanorama;
		cylindricalPanorama.applyCustomCylindricalWarping(image_names);
	}

	else if (panoType == SPHERICAL) {
		if (argc == 5) {
			try {
				double hfov = stod(argv[3]);
				double vfov = stod(argv[4]);
				CustomSphericalPanorama sphericalPanorama;
				sphericalPanorama.applyCustomSphericalWarping(image_names, hfov, vfov);
			}
			catch(exception e){
				input_output.writeFieldOfViewError();
			}			
		}
		else {
			input_output.provideCorrectInputError();
		}
	}

	return 0;
}

#else
int main(int argc, char** argv) {
	cout << " Opencv Configuration error. Please check your version OR please be sure that you have configured opencv_contrib modules ! " << endl;
	return -1;
}
#endif