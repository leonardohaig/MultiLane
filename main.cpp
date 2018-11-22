#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>


#include "LaneDetection.h"

int main()
{
	
	// input parameter
	bool verbose_lm_detction = true;
	bool verbose_seed_gen = false;
	bool verbose_run_crf = false;
	bool verbose_validating = true;
	bool verbose = verbose_lm_detction | verbose_seed_gen | verbose_run_crf | verbose_validating;

	//<-------------- Common Variables definition & initialization --------------> 
	//std::string img_path = "../data/20120322142840/";
	std::string img_path = "/home/liheng/CLionProjects/kitti/01/left/";

	LaneDetection ld = LaneDetection();
	//std::string img_name = (std::string(img_path)).append(std::to_string(1)).append(".jpg");
	std::string img_name = (std::string(img_path)).append("000000.png");
	
	// initilaize
	if (!ld.initialize_variable(img_name)) {
		return 0;
	}
	
	// process
	for(int ff=1;ff<1000;ff++){

		//std::string img_name = (std::string(img_path)).append(std::to_string(ff)).append(".jpg");
		char path[256];
		sprintf(path,"/home/liheng/CLionProjects/kitti/01/left/%06d.png",ff);
		img_name = path;

		// Initlaization
		if (!ld.initialize_Img(img_name)) {
			continue;
		}
		
		// detecting lane markings
		ld.lane_marking_detection(verbose_lm_detction);

		// supermarking generation and low-level association
		ld.seed_generation(verbose_seed_gen);

		// CRF graph configuration & optimization using hungarian method
		ld.graph_generation(verbose_run_crf);
		
		// validating
		ld.validating_final_seeds(verbose_validating);
	

		if (verbose) {
			cv::waitKey(0);
		}
	}

	//ld.~LaneDetection();

    return  0;
	
}