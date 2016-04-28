#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define MAP_FILE ROOT_PATH "/data/map_0.png"
#define WINDOW_SIZE 700

void compute_skeleton(Mat& src, Mat& dst)
{
	Mat skeleton = src.clone();
	Mat skel(skeleton.size(), src.type(), Scalar(0));
	Mat temp(skeleton.size(), src.type());
	Mat eroded(skeleton.size(), src.type());
	
	Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
	 
	bool done = false;		
	while (!done)
	{
		erode(skeleton, eroded, element);
		dilate(eroded, temp, element); // temp = open(skeleton)
		subtract(skeleton, temp, temp);
		bitwise_or(skel, temp, skel);
		eroded.copyTo(skeleton);
		done = (countNonZero(skeleton) == 0);
	}
	dst = skel.clone();	
}

int main(int argc, char const *argv[])
{
	// Read map image (from argument or predefined file)
	Mat map;
	if (argc > 1)
	{
		map = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	}
	else
	{
		map = imread(MAP_FILE, CV_LOAD_IMAGE_GRAYSCALE);
	}
	if (map.empty())
	{
		cout << "Could not load the image\n";
		return 1;
	}
	// Resize to WINDOW_SIZE
	resize(map, map, Size(WINDOW_SIZE,WINDOW_SIZE), 0, 0, CV_INTER_LINEAR);

	imshow("original map", map);
	waitKey(0);

	Mat bin_map;
	threshold(map, bin_map, 230, 255, THRESH_BINARY);
	imshow("binary map", bin_map);
	waitKey(0);

	// Erode and dilate. Why not?
	// Erode 8 times (make obstacles bigger)
	erode(bin_map, bin_map, Mat(), Point(-1,-1), 8);
	imshow("binary map", bin_map);
	waitKey(0);

	// Dilate 4 times (make free space bigger)
	dilate(bin_map, bin_map, Mat(), Point(-1,-1), 4);
	imshow("binary map", bin_map);
	waitKey(0);

	
	/*
	// First approach: Skeleton from dilate + erode
	Mat skeleton;
	compute_skeleton(bin_map, skeleton);
	imshow("skeleton", skeleton);
	waitKey(0);
	*/	

	// Second approach: Contours to get polygons
	vector<vector<Point>> contours;
	Mat canny_map;
	Canny(bin_map, canny_map, 50, 200, 5, false);

	findContours(canny_map, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
	cout << "Contours size: " << contours.size() << endl;
	
	Mat frame_contours = Mat::zeros(bin_map.size(), CV_8UC3);
	vector<vector<Point>> new_contours;
	for (size_t i = 0; i < contours.size(); ++i)
	{
		cout << "Contour " << i << endl;
		cout << "\tarea: " << contourArea(contours[i], true) << endl;
		cout << "\tlength: " << arcLength(Mat(contours[i]), true) << endl;
		cout << "\tpoints: " << contours[i].size() << endl;
		if (contourArea(contours[i], true) < 0)
		{
			new_contours.push_back(contours[i]);
			drawContours(frame_contours, contours, i, Scalar(0,255,0), 1, 8, 0, 0, Point(0,0));
		}
		else
		{
			cout << "Positive area. Skipping..." << endl;
		}
	}
	imshow("Contours", frame_contours);
	waitKey(0);

	Mat frame_mixed = bin_map.clone();
	cvtColor(frame_mixed, frame_mixed, CV_GRAY2BGR);
	
	// Get approximate polygons to work with less points and simpler forms
	vector<vector<Point>> approx_contours;
	approx_contours.resize(new_contours.size());
	double eps = 0.002; // Approximation accuracy. % difference between original arclength and new
	frame_contours = Mat::zeros(bin_map.size(), CV_8UC3);
	for (int i = 0; i < new_contours.size(); ++i)
	{
		approxPolyDP( Mat(new_contours[i]), approx_contours[i], arcLength(Mat(new_contours[i]), true)*eps, true);
		cout << "Approx contour " << i << endl;
		cout << "\tarea: " << contourArea(approx_contours[i], true) << endl;
		cout << "\tlength: " << arcLength(Mat(approx_contours[i]), true) << endl;
		cout << "\tpoints: " << approx_contours[i].size() << endl;
		drawContours(frame_contours, approx_contours, i, Scalar(0,0,255), 1, 8, 0, 0, Point(0,0));
		drawContours(frame_mixed, approx_contours, i, Scalar(0,0,255), 2, 8, 0, 0, Point(0,0));
	}
	imshow("Contours", frame_contours);
	imshow("Mixed", frame_mixed);
	waitKey(0);


	// Compute voronoi?

	// Second approach: Use pointPolygonTest to build distance maps to each obstacle and take the minimum for each point
	int num_contours = approx_contours.size();
	Mat distance_maps[num_contours];
	Mat draw_distance_map(bin_map.size(), CV_8UC3);
	float mins[num_contours];
	float maxs[num_contours];
	for (int i = 0; i < num_contours; ++i)
	{
		distance_maps[i] = Mat(bin_map.size(), CV_32FC1);
	}

	float distance = 0;
	int region = -1;
	float min_dist = 99999;
	vector<Point> voronoi_points;
	for (int i = 0; i < bin_map.rows; ++i)
	{
		for (int j = 0; j < bin_map.cols; ++j)
		{
			region = -1;
			min_dist = 9999;
			if (bin_map.at<uchar>(i,j) == 255)
			{
				for (int k = 0; k < num_contours; ++k)
				{
					distance = abs(pointPolygonTest(approx_contours[k], Point(j,i), true));
					distance_maps[k].at<float>(i,j) = distance;
					//cout << "Dist: " << distance << ", " << distance_maps[k].at<float>(i,j) << endl;
					if (abs(distance - min_dist) < 1)
					{
						// Voronoi graph point
						region = num_contours;
						voronoi_points.push_back(Point(j,i));
					}
					else if (distance < min_dist)
					{
						min_dist = distance;
						region = k;
					}
				}
				/*
				switch (region)
				{
					case 0:
						draw_distance_map.at<Vec3b>(i,j) = Vec3b(255,0,0);
						break;
					case 1:
						draw_distance_map.at<Vec3b>(i,j) = Vec3b(0,255,0);
						break;
					case 2:
						draw_distance_map.at<Vec3b>(i,j) = Vec3b(0,0,255);
						break;
					default:
						break;
				}
				*/
			}
		}
	}
	draw_distance_map = map.clone();
	cvtColor(draw_distance_map, draw_distance_map, CV_GRAY2BGR);
	for (int i = 0; i < voronoi_points.size(); ++i)
	{
		draw_distance_map.at<Vec3b>(voronoi_points[i]) = Vec3b(0,0,255);
	}

	vector<Point> voronoi_approx;
	eps = 0.0009;
	approxPolyDP( Mat(voronoi_points), voronoi_approx, arcLength(Mat(voronoi_points), true)*eps, false);

	cout << "Initial voronoi points: " << voronoi_points.size() << endl;
	cout << "Initial voronoi length: " << arcLength(Mat(voronoi_points), true) << endl;
	cout << "Approx voronoi points: " << voronoi_approx.size() << endl;
	Mat draw_voronoi = map.clone();
	cvtColor(draw_voronoi, draw_voronoi, CV_GRAY2BGR);
	for (int i = 0; i < voronoi_approx.size(); ++i)
	{
		circle(draw_voronoi, voronoi_approx[i], 5, Scalar(0,0,255), CV_FILLED, 8);
	}
	
	/*
	// Visualize distance transform
	for (int i = 0; i < num_contours; ++i)
	{
		normalize(distance_maps[i], distance_maps[i], 0, 1, NORM_MINMAX);
		subtract(1, distance_maps[i], distance_maps[i]);
	}
	imshow("distance 0", distance_maps[0]);
	imshow("distance 1", distance_maps[1]);
	*/
	
	imshow("draw_distances", draw_distance_map);
	waitKey(0);
	imshow("Voronoi", draw_voronoi);
	waitKey(0);

	destroyAllWindows();
	return 0;
}