#include "ACOPlanner.cpp"
#include "AStar.h"
#include "Node.h"

#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define MAP_FILE ROOT_PATH "/data/map_0.png"
#define WINDOW_SIZE 700


// VARIABLES GLOBALES
// Para el primer trackbard
Mat dilero, before_dilero; // Imagen erosionada y dilatada; Se pone en global para el manejo del trackpad
const int valor_slider_max = 10;
const double eps_max = 20;  // Maximo tamaño para el sliderl.
int valor_slider1, valor_slider2; // Valor de los slider.
//Para el segundo trackbard
Mat bin_map, frame_mixed;
int eps_track; // Approximation accuracy. % difference between original arclength and new
vector<vector<Point>> new_contours;
vector<vector<Point>> approx_contours;
// Variables globales Alvaro
vector<Point> centroids;
Mat voronoi_distance_ij;
Mat centroids_bin_map;
Mat initMap;
Point init,fin;
int indexInit, indexFin;
vector<int> path;



Node* nodeIni;
Node* nodeFin;
std::list<Node*> camino;
std::vector<Node*> grafos;

/*
 * Funcion callback del trackpad de erosion y dilatacion
 */
void on_trackbar_dilero( int, void* )
{	
	erode(before_dilero, dilero, Mat(), Point(-1,-1), valor_slider1);
	dilate(dilero, dilero, Mat(), Point(-1,-1), valor_slider2);
	std::cout<<"La mierda de Pou me motiva."<<std::endl;
	cout<<"Valor de la Erosion: "<< valor_slider1 << "; Valor Dilatacion: " << valor_slider2 << endl;
 	imshow( "Dilatacion_Erosion", dilero );
}

/*
 * Funcion callback para simplificar en poligonos mas simples.
 */
void on_trackbar_poligon(int, void*)
{
	frame_mixed = bin_map.clone();
	cvtColor(frame_mixed, frame_mixed, CV_GRAY2BGR);

	double eps = eps_track * 0.001;
	approx_contours.resize(new_contours.size());
	int num_contours = 0;
	//Mat frame_contours2 = Mat::zeros(frame_mixed	.size(), CV_8UC3);
	for (int i = 0; i < new_contours.size(); ++i)
	{
		approxPolyDP( Mat(new_contours[i]), approx_contours[i], arcLength(Mat(new_contours[i]), true)*eps, true);
		cout << "Approx contour " << i << endl;
		cout << "\tarea: " << contourArea(approx_contours[i], true) << endl;
		cout << "\tlength: " << arcLength(Mat(approx_contours[i]), true) << endl;
		cout << "\tpoints: " << approx_contours[i].size() << endl;
		num_contours += approx_contours[i].size();
		//drawContours(frame_contours2, approx_contours, i, Scalar(0,0,255), 1, 8, 0, 0, Point(0,0));
		drawContours(frame_mixed, approx_contours, i, Scalar(0,0,255), 2, 8, 0, 0, Point(0,0));
	}
	//imshow("Contours", frame_contours);
	cout << "EPS: " << eps << "Numero de contornos:" << num_contours;
	imshow("Mixed", frame_mixed);
}

void drawCentroids()
{
	centroids_bin_map = initMap.clone();
    cvtColor(centroids_bin_map, centroids_bin_map, CV_GRAY2RGB);
    for (int i = 0; i < centroids.size(); i++)
    {
    	circle( centroids_bin_map, centroids[i], 4, Scalar(255,0,0), CV_FILLED, 8, 0 );
    }
    
    for (int i = 0; i < path.size(); ++i)
    {
    	circle( centroids_bin_map, centroids[path[i]], 4, Scalar(0,100,255), CV_FILLED, 8, 0 );
    }

    for (int i = 0; i < centroids.size(); i++)
    {
    	if(centroids[i].x == init.x && centroids[i].y == init.y)
    		circle( centroids_bin_map, centroids[i], 4, Scalar(0,255,0), CV_FILLED, 8, 0 );
    	else if(centroids[i].x == fin.x && centroids[i].y == fin.y)
    		circle( centroids_bin_map, centroids[i], 4, Scalar(0,0,255), CV_FILLED, 8, 0 );
    }

    imshow("centroids", centroids_bin_map);
}

void calculatePath()
{
	if(indexInit>-1 && indexFin>-1)
	{
	    double path_distance = ACOPlanner(centroids, indexInit, indexFin, voronoi_distance_ij, path);
	    cout << "Path distance: " << path_distance << ", size: " << path.size() << endl;
	    cout << "Path: ";
	    for (int i = 0; i < path.size(); ++i)
	    {
	    	cout << path[i] << " - ";
	    }
	    cout << endl;
	    drawCentroids();
	}
}

void on_mouse(int evt, int x, int y, int flags, void* param) {
	float calc=false;
    if(evt == CV_EVENT_LBUTTONDOWN) {
        //Point* ptPtr = (Point*)param;
        printf("Point: (%d , %d)\n", x, y);
        for(int i=0; i<centroids.size(); i++)
        {
        	if((x<=centroids[i].x +5 && x>=centroids[i].x -5) && (y<=centroids[i].y +5 && y>=centroids[i].y -5))
        	{
        		if(init.x == -1 && init.y==-1)
        		{
        			indexInit=i;
        			init=centroids[i];
        		}
        		else if(fin.x == -1 && fin.y ==-1)
        		{
        			indexFin=i;
        			fin=centroids[i];
        			calc=true;
        		}
        		else
        		{
        			indexInit=i;
        			indexFin=-1;
        			init=centroids[i];
        			fin=Point(-1,-1);
        			path.clear();
        		}
        		printf("Centroid: %d \n", i);
        		drawCentroids();
        		if(calc)
        			calculatePath();
        	}
        }
    }
}

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


void search_centroids(Mat& points, Point start, vector<Point>& visited_corners, vector<Point>& current_group)
{
	visited_corners.push_back(start);
	current_group.push_back(start);
    
    const int mask_size = 10;
    Point mask_start, mask_end;

	mask_start.y = max(0, start.y - mask_size);
	mask_end.y = min(points.rows, start.y + mask_size);
	mask_start.x = max(0, start.x - mask_size);
	mask_end.x = min(points.cols, start.x + mask_size);

	for (int i = mask_start.y; i < mask_end.y; ++i)
	{
		for (int j = mask_start.x; j < mask_end.x; ++j)
		{
			Point current_point = Point(j,i);
			if (points.at<uchar>(i,j) == 0)
			{
				// Found obstacle. Create a point in the opposite side of the mask to modify the centroid and avoid the obstacle
				Point opposite;
				int diff_y = start.y - i;
				int diff_x = start.x - j;
				if (diff_y > 0) opposite.y = mask_end.y - diff_y;
				else opposite.y = mask_start.y - diff_y;
				if (diff_x > 0) opposite.x = mask_end.x - diff_x;
				else opposite.x = mask_start.x - diff_x;

				if (points.at<uchar>(opposite) != 0)
				{
					// Do not add points where there are obstacles
					current_group.push_back(opposite);
					visited_corners.push_back(opposite);
				}
			}
			if (points.at<uchar>(i,j) == 100 && find(visited_corners.begin(), visited_corners.end(), current_point) == visited_corners.end())
			{
				// Found another corner to compute
				search_centroids(points, current_point, visited_corners, current_group);
			}
		}
	}
}

Point compute_centroid(vector<Point>& current_group)
{
	Point2f centroid = Point(0,0);
	for (int i = 0; i < current_group.size(); ++i)
	{
		centroid.x += current_group[i].x;
		centroid.y += current_group[i].y;
	}
	centroid.x /= current_group.size();
	centroid.y /= current_group.size();

	return centroid;
}

void compute_voronoi_graph(Mat& obstacles, vector<Point>& nodes, Mat& distances)
{
	distances.setTo(0.0);
	for (int i = 0; i < nodes.size(); ++i)
	{
		for (int j = i+1; j < nodes.size(); ++j)
		{
			// Check if there is connection between points
			LineIterator it(obstacles, nodes[i], nodes[j], 8);
			bool obstacle_in_line = false;
			for(int k = 0; k < it.count; k++, ++it)
			{
				Point linePoint = it.pos();
				if (obstacles.at<uchar>(linePoint) == 0)
				{
					obstacle_in_line = true;
					break;
				}
			}
			// Compute distance
			if (obstacle_in_line)
			{
				distances.at<double>(i,j) = -1;
				distances.at<double>(j,i) = -1;
			}
			else
			{
				double d = euclideanDist(nodes[i], nodes[j]);
				distances.at<double>(i,j) = d;
				distances.at<double>(j,i) = d;
			}
		}
	}
}



int main(int argc, char const *argv[])
{
	init=Point(-1,-1);
	fin=Point(-1,-1);
	// Read map image (from argument or predefined file)
	//Mat map;
	if (argc > 1)
	{
		initMap = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	}
	else
	{
		initMap = imread(MAP_FILE, CV_LOAD_IMAGE_GRAYSCALE);
	}
	if (initMap.empty())
	{
		cout << "Could not load the image\n";
		return 1;
	}
	// Resize to WINDOW_SIZE
	resize(initMap, initMap, Size(WINDOW_SIZE,WINDOW_SIZE), 0, 0, CV_INTER_LINEAR);

	//imshow("original map", map);
	//waitKey(0);


	threshold(initMap, bin_map, 230, 255, THRESH_BINARY);
	//imshow("binary map", bin_map);
	//waitKey(0);


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	/*
	 *	EROSIONAMOS Y DILATAMOS.
	 *	Aplicamos erosion y dilatacion con trackpad mirar funcion callback superior.
	 *	Erode and dilate. Why not?
	 */

	// Clonamos el mapa para  
	before_dilero = bin_map.clone(); // Clonamos binmap antes de meterselo a diletacion/erosion con trackpad
	namedWindow("Dilatacion_Erosion", WINDOW_AUTOSIZE);
	// Create Trackbars
	std::string TrackbarErosionName = "Erosion";
	std::string TrackbarDilatacionName = "Dilatacion";
	createTrackbar( TrackbarErosionName, "Dilatacion_Erosion", &valor_slider1, valor_slider_max, on_trackbar_dilero );
	createTrackbar( TrackbarDilatacionName, "Dilatacion_Erosion", &valor_slider2, valor_slider_max, on_trackbar_dilero );
	// Show some stuff
	on_trackbar_dilero( valor_slider1, 0 );
	// Dilate 4 times (make free space bigger)
	waitKey(0);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -



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
	//imshow("Contours", frame_contours);
	//waitKey(0);


	
// - - - - - - - - - - - - - - - -- - -  - -- - - - - - - - - - - - - - - -

	/*
	 * ES: Cogemos poligonos aproximados para trabajar con menos puntos y formas mas simples.
	 * EN: Get approximate polygons to work with less points and simpler forms
	 * Se realiza en el callback del trackpad.
	 * TrackPad : El porcentaje original entre el poligono original y el aproximado
	 */
	frame_mixed = bin_map.clone();
	cvtColor(frame_mixed, frame_mixed, CV_GRAY2BGR);
	namedWindow("Mixed", WINDOW_AUTOSIZE);
	/// Create Trackbars
	std::string TrackbarName = "Precision";
	createTrackbar( TrackbarName, "Mixed", &eps_track, valor_slider_max, on_trackbar_poligon );
	/// Show some stuff
	on_trackbar_poligon( eps_track, 0 );
	// Esperamos iteracción con teclas
	waitKey(0);

// - - - - - - - - - - - - - - - -- - -  - -- - - - - - - - - - - - - - - -



	// Split contours into lines, so each line is considered a different obstacle.
	vector<vector<Point>> obstacles_contours;
	int contour_points;
	//Mat wtf = bin_map.clone();
	//cvtColor(wtf, wtf, CV_GRAY2BGR);
	for (int i = 0; i < approx_contours.size(); ++i)
	{
		vector<Point> next_obstacles;
		contour_points = approx_contours[i].size();
		for (int j = 0; j < contour_points; ++j)
		{
			// First point: approx_contours[i][j];
			// Second point: approx_contours[i][(j+1) % contour_points];
			next_obstacles.push_back(approx_contours[i][j]);
			next_obstacles.push_back(approx_contours[i][(j+1) % contour_points]);
			
			obstacles_contours.push_back(next_obstacles);
			//line(wtf, next_obstacles[0], next_obstacles[1], Scalar(0,0,255), 2, 8);
			//imshow("wtf", wtf);
			//waitKey(0);
			next_obstacles.clear();
		}
	}

	/* Compute voronoi */

	// Main approach: Use pointPolygonTest to build distance maps to each obstacle and take the minimum for each point
	int num_contours = obstacles_contours.size();
	cout << "Final contours: " << num_contours << endl;

	// Distance maps to each contour. Only for visualization and debug.
	vector<Mat> distance_maps;
	distance_maps.resize(num_contours);
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
			// Store the distances from a point to each obstacle to sort them (and the obstacle index)
			vector<pair<float,int>> point_obstacle_dist;
			point_obstacle_dist.resize(num_contours);
			if (bin_map.at<uchar>(i,j) == 255)
			{
				for (int k = 0; k < num_contours; ++k)
				{
					distance = abs(pointPolygonTest(obstacles_contours[k], Point(j,i), true));
					distance_maps[k].at<float>(i,j) = distance;
					point_obstacle_dist[k] = make_pair(distance, k);
					//cout << "Dist: " << distance << ", " << distance_maps[k].at<float>(i,j) << endl;
				}
				// Sort distances (lowest first)
				sort(point_obstacle_dist.begin(), point_obstacle_dist.end());
				if (num_contours >= 2)
				{
					if (abs(point_obstacle_dist[0].first - point_obstacle_dist[1].first) < 1)
					{
						// At least the 2 closest obstacles are at the same distance
						voronoi_points.push_back(Point(j,i));
						region = num_contours;
					}
					else
					{
						// 1 obstacle closer then the others
						region = point_obstacle_dist[0].second;
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

	Mat voronoi_map = initMap.clone();
	cvtColor(voronoi_map, voronoi_map, CV_GRAY2BGR);
	for (int i = 0; i < voronoi_points.size(); ++i)
	{
		voronoi_map.at<Vec3b>(voronoi_points[i]) = Vec3b(0,0,255);
	}	

	/*
	// Visualize distance transform (only for debug)
	for (int i = 0; i < num_contours; ++i)
	{
		normalize(distance_maps[i], distance_maps[i], 0, 1, NORM_MINMAX);
		subtract(1, distance_maps[i], distance_maps[i]);
		//imshow("distance", distance_maps[i]);
		//waitKey(0);
	}
	destroyWindow("distance");
	*/
	
	//imshow("voronoi points", voronoi_map);
	//waitKey(0);
	//imshow("Voronoi", draw_voronoi);
	//waitKey(0);


	// Try to compute the real voronoi graph (nodes and edges) from the set of voronoi points
	voronoi_map = Mat(bin_map.size(), CV_8UC1, Scalar(0));
	for (int i = 0; i < voronoi_points.size(); ++i)
	{
		voronoi_map.at<uchar>(voronoi_points[i]) = 255;
	}
	imshow("voronoi bin", voronoi_map);
	waitKey(0);

/*
	// Remove filled areas
	Canny(voronoi_map, canny_map, 100, 200, 3, false);
	imshow("voronoi canny", canny_map);
	waitKey(0);
*/



	// Compute skeleton over voronoi points to remove filled areas and detect corners
	Mat skeleton;
	compute_skeleton(voronoi_map, skeleton);
    //imshow("skeleton", skeleton);
    //waitKey(0);

    Mat dst_norm, dst_norm_scaled, corners;

    cornerHarris( skeleton, corners, 2, 3, 0.01, BORDER_DEFAULT );
    /// Normalizing
    normalize( corners, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );

    //imshow("corners", dst_norm_scaled);
    //waitKey(0);

    Mat newImg = Mat::zeros(dst_norm_scaled.size(), CV_8UC3);
    Mat final = initMap.clone();
    cvtColor(final, final, CV_GRAY2BGR);
    vector<Point> corners_list;
    Mat puntitos = bin_map.clone();

    /// Drawing a circle around corners
    for( int j = 0; j < dst_norm.rows ; j++ )
    { for( int i = 0; i < dst_norm.cols; i++ )
        {
            if( (int) dst_norm.at<float>(j,i) > 40 && (int) dst_norm.at<float>(j,i) < 180 )
            {
            	corners_list.push_back(Point(i,j));
            	puntitos.at<uchar>(j,i) = 100;
                circle( final, Point( i, j ), 2,  (255,255,255), 1, 8, 0 );
            }
        }
    }
    cout << "Puntitos: " << corners_list.size() << endl;
    imshow("voronoi points", final);
    imshow("puntitos", puntitos);
    waitKey(0);



    // Take the detected corners and compute their centroids
    vector<Point> visited_corners;
    //vector<Point> centroids;
    vector<Point> current_group;

    Mat centroids_map = bin_map.clone();
    cvtColor(centroids_map, centroids_map, CV_GRAY2BGR);
    for (int k = 0; k < corners_list.size(); ++k)
    {
    	
    	current_group.clear();
    	if (find(visited_corners.begin(), visited_corners.end(), corners_list[k]) == visited_corners.end())
    	{
    		// If the point has not been visited
    		// Get the group of points near to it
    		search_centroids(puntitos, corners_list[k], visited_corners, current_group);
    		// Compute centroid of points in current_group
    		Point centroid = compute_centroid(current_group);
    		cout << "Group size: " << current_group.size() << endl;
    		//Point centroid;
    		centroids.push_back(centroid);
    		grafos.push_back(new Node(Point2f(centroid.x,centroid.y),nullptr,0.0f));
    		circle( centroids_map, centroid, 4, (255,255,255), CV_FILLED, 8, 0 );
    	}
    }
    cout << "Final centroids: " << centroids.size() << endl;
    imshow("centroids", centroids_map);
    waitKey(0);


    // Compute the voronoi graph, being the centroids the nodes

    // Matrix to store the distance between each pair of nodes
    voronoi_distance_ij = Mat_<double>(centroids.size(), centroids.size());
    compute_voronoi_graph(bin_map, centroids, voronoi_distance_ij);

    cout << voronoi_distance_ij << endl;

    // Draw graph
    centroids_bin_map = bin_map.clone();
    centroids_map = initMap.clone();
    cvtColor(centroids_bin_map, centroids_bin_map, CV_GRAY2BGR);
    cvtColor(centroids_map, centroids_map, CV_GRAY2BGR);
    for (int i = 0; i < centroids.size(); ++i)
    {
    	for (int j = i+1; j < centroids.size(); ++j)
    	{
    		if (voronoi_distance_ij.at<double>(i,j) > 0)
    		{
    			line(centroids_map, centroids[i], centroids[j], Scalar(0,255,0), 1, 8);
    			line(centroids_bin_map, centroids[i], centroids[j], Scalar(0,255,0), 1, 8);
    		}
    	}
    	circle( centroids_map, centroids[i], 4, (255,255,255), CV_FILLED, 8, 0 );
    	circle( centroids_bin_map, centroids[i], 4, (255,255,255), CV_FILLED, 8, 0 );
    }
    imshow("centroids", centroids_map);
    imshow("centroids bin", centroids_bin_map);
    waitKey(0);

    setMouseCallback("centroids",on_mouse);

    waitKey(0);

	destroyAllWindows();
	return 0;
}