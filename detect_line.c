

char line_detection_kernel [9] = {-1, 0, 1, -1, 0, 1, -1, 0, 1} ;

#define IMG_HEIGHT 320
#define SAMPLES_FIRST_LINE 32
#define SAMPLES_LAST_LINE 300
#define NB_LINES_SAMPLED 32
#define SAMPLES_SPACE ((SAMPLES_LAST_LINE - SAMPLES_FIRST_LINE)/NB_LINES_SAMPLED)

typedef struct line{
	float a ;
	float b ;
}line;


void kernel_line(IplImage * img, char * kernel ,int * kernel_response, int v){
	unsigned int i ;
	unsigned int kernel_sum = 0 ;
	kernel_response[0] = 0;
	for(i = 1 ; i < (img->width-1) ; i ++){
		int response = 0 ;
		response += kernel[0]* img->imageData[(v-1)*img->widthStep+(i-1)];
		response += kernel[1]* img->imageData[(v-1)*img->widthStep+(i)];
		response += kernel[2]* img->imageData[(v-1)*img->widthStep+(i+1)];
		
		response += kernel[3]* img->imageData[(v)*img->widthStep+(i-1)];
		response += kernel[4]* img->imageData[(v)*img->widthStep+(i)];
		response += kernel[5]* img->imageData[(v)*img->widthStep+(i+1)];
		
		response += kernel[6]* img->imageData[(v+1)*img->widthStep+(i-1)];
		response += kernel[7]* img->imageData[(v+1)*img->widthStep+(i)];
		response += kernel[8]* img->imageData[(v+1)*img->widthStep+(i+1)];
		
		kernel_response[i] = reponse ;
		
	}
	kernel_response[i] = 0;
}

int detect_line(IplImage * img, line * l ){
	unsigned int i ;
	unsigned int sample_space
	int ** sampled_lines = malloc(NB_LINES_SAMPLED * sizeof(int*));
	for(i = 0 ; i < NB_LINES_SAMPLED ; i ++){
		sampled_lines[i] = malloc(img->width*sizeof(int));
		kernel_line(img, line_detection_kernel ,int * (sampled_lines[i], (SAMPLES_FIRST_LINE + i * (SAMPLES_SPACE)));
	}
	//We approximate by a three degree polynom
	//We use triplet of points to compute the coefficients,  	
	
	// 1) for each line sampled, find a max folowed by a min to identify the line center and compute its pos
	// 2) use multiple measurements to approximate a line

}

//Compute the line equation in the ground plane (need a calibrated camera system ...)
//Compute an interception curve in the ground plane
//Compute steering from interception curve and apply to vehicle (compute speed based on steering to avoid drift)
