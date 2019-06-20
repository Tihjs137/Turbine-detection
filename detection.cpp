/**
  @file     detection.h
  @project  Turbine detection
  @author   Thijs de Jong
  @email    thijsdejong21@gmail.com
  @date     16th of May, 2019
*/


//class header
#include "detection.hpp"

//Namespace
using namespace std; 
using namespace cv;
using namespace cv::xfeatures2d; 

Detector::Detector()
{
        //Background removal
        bw2_Treshold    = 180;
        
        //Hough line finder
        bw4_Rho         = 5;
        bw4_Theta       = 180*10;
        bw4_Treshold    = 120;
        bw4_minLinLength       = 150;
        bw4_maxLineGap         = 30;

        //create trackbars
        namedWindow(    "bw2 mask",1);
        createTrackbar( "bw2_Treshold",     "bw2 mask", &bw2_Treshold, 255, NULL );

        namedWindow(    "bw4: Hough lines",1);
        createTrackbar( "bw4_Rho",          "bw4: Hough lines", &bw4_Rho,       10, NULL );
        createTrackbar( "bw4_Theta",        "bw4: Hough lines", &bw4_Theta,     360, NULL );
        createTrackbar( "bw4_Treshold",     "bw4: Hough lines", &bw4_Treshold,  255, NULL );
        createTrackbar( "bw4_minLinLength",          "bw4: Hough lines", &bw4_minLinLength,       200, NULL );
        createTrackbar( "bw4_maxLineGap",          "bw4: Hough lines", &bw4_maxLineGap,       30, NULL );

        //created named windows, for the trackbars
        window_capture_name = "Video Capture";
        window_detection_name = "Object Detection";

        //Set HSV values for trackbar
        low_H = 107, low_S = 54, low_V = 105;
        high_H = 154, high_S = 133, high_V = 147;

        namedWindow(window_detection_name);
        // Trackbars to set thresholds for HSV values
        createTrackbar("Low H" , window_detection_name, &low_H , max_value_H, NULL);
        createTrackbar("High H", window_detection_name, &high_H, max_value_H, NULL);
        createTrackbar("Low S" , window_detection_name, &low_S , max_value  , NULL);
        createTrackbar("High S", window_detection_name, &high_S, max_value  , NULL);
        createTrackbar("Low V" , window_detection_name, &low_V , max_value  , NULL);
        createTrackbar("High V", window_detection_name, &high_V, max_value  , NULL);
    };

void Detector::showVideo(Mat frame)
{
        
    imshow("live",frame);

}

void Detector::locate(Mat frame, vector<Point2d> image_points) 
{
    /*
    @brief : This funtion calculates the rotation and translation based on points 
    @reference : https://www.learnopencv.com/head-pose-estimation-using-opencv-and-dlib/

    */


    // image_points.push_back( cv::Point2d(257, 394) );    
    // image_points.push_back( cv::Point2d(371, 295) );    
    // image_points.push_back( cv::Point2d(268, 288) );     
    // image_points.push_back( cv::Point2d(165, 279) ); 
    // image_points.push_back( cv::Point2d(278, 186) ); 

    // Fill image_points using blob detection

    // image_points.push_back( cv::Point2d(keypoints[0].pt.x, keypoints[0].pt.y) );    
    // image_points.push_back( cv::Point2d(keypoints[1].pt.x, keypoints[1].pt.y) );    
    // image_points.push_back( cv::Point2d(keypoints[2].pt.x, keypoints[2].pt.y) );    
    // image_points.push_back( cv::Point2d(keypoints[3].pt.x, keypoints[3].pt.y) ); 
    // image_points.push_back( cv::Point2d(keypoints[4].pt.x, keypoints[4].pt.y) ); 

    // 3D model points.
    std::vector<cv::Point3d> model_points;
    model_points.push_back(cv::Point3d(  0.0f,  0.0f, 0.0f));  //Center           
    model_points.push_back(cv::Point3d(  25.0f,25.0f, 0.0f));  //right top         
    model_points.push_back(cv::Point3d(-25.0f,-25.0f, 0.0f));  //left bottom    
    model_points.push_back(cv::Point3d(-25.0f, 25.0f, 0.0f));  //left top
    model_points.push_back(cv::Point3d( 25.0f,-25.0f, 0.0f));  //right bottom     
    
    
    // Camera internals
    double focal_length = frame.cols; // Approximate focal length.
    Point2d center = cv::Point2d(frame.cols/2,frame.rows/2);
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion
    
    cout << "Camera Matrix " << endl << camera_matrix << endl ;
    
    
    // Solve for pose 
    //FIXME: error upons video stream, after few frames
    cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector, false); //disable iterative calc

    
    // Project a axis onto the image plane.
    
    
    vector<Point3d> nose_end_point3D;
    vector<Point2d> nose_end_point2D;
    nose_end_point3D.push_back(Point3d(100,0,0));
    nose_end_point3D.push_back(Point3d(0,100,0));
    nose_end_point3D.push_back(Point3d(0,0,100));
    
    projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
    
    
    for(int i=0; i < image_points.size(); i++)
    {
        circle(frame, image_points[i], 3, Scalar(0,0,255), -1);
    }
    
    cv::line(frame,image_points[0], nose_end_point2D[0], cv::Scalar(255,0,0), 2);
    cv::line(frame,image_points[0], nose_end_point2D[1], cv::Scalar(0,255,0), 2);
    cv::line(frame,image_points[0], nose_end_point2D[2], cv::Scalar(0,0,255), 2);
    
    // int str = (rotation_vector.at<float>(0));
    // stringstream iss;
    // iss << str;
    // putText(frame, iss.str(), Point2d(10,20), 1, 2, Scalar(255,0,0), 2);
    cout << "Rotation Vector " << endl << rotation_vector << endl;
    cout << "Translation Vector" << endl << translation_vector << endl;
    
    cout <<  nose_end_point2D << endl;

    imshow("PNP - output", frame);
}

void Detector::locate(Mat frame) 
{   
    //C++: bool solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, 
    //          InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int flags=ITERATIVE )¶
    
    // show live and wait for a key with timeout long enough to show images
    imshow("Live", frame);

    // -------- gray scaling ----------

    


    

    Mat frame_HSV; Mat frame_threshold;
    // Convert from BGR to HSV colorspace
    cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
    // Detect the object based on HSV Range Values
    inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
    
    imshow("HSV",frame_HSV);
    imshow(window_detection_name, frame_threshold);

    // -------- Blob detection ----------

    // Set up the detector with default parameters.
    
    
    // Detect blobs.
    std::vector<KeyPoint> keypoints;
    

    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;
    
    // Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 255;
    
    // Filter by Area.
    params.filterByArea = false;
    params.minArea = 1500;
    
    // Filter by Circularity
    params.filterByCircularity = false;
    params.minCircularity = 0.8;
    
    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.87;
    
    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;
    
    //Invert the image
    Mat beforeBlob = frame_threshold < 200;

    //Run the blob detector
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params); 
    detector->detect( beforeBlob, keypoints );

    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    Mat im_with_keypoints;
    drawKeypoints( beforeBlob, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    
    // Show blobs
    imshow("keypoints", im_with_keypoints );

    //Debug print
    for( int i = 0 ; i < keypoints.size(); i++)
    {
        cout << "Blob " << i << " : " << keypoints[i].pt << "\t";

    }
    
    cout << "\n\n";


    //PnP magic ...
    std::vector<cv::Point2d> image_points;

    if(keypoints.size() == 5)
    {
        
        // image_points.push_back( cv::Point2d(257, 394) );    
        // image_points.push_back( cv::Point2d(371, 295) );    
        // image_points.push_back( cv::Point2d(268, 288) );     
        // image_points.push_back( cv::Point2d(165, 279) ); 
        // image_points.push_back( cv::Point2d(278, 186) ); 

        //Fill image_points using blob detection

        image_points.push_back( cv::Point2d(keypoints[0].pt.x, keypoints[0].pt.y) );    
        image_points.push_back( cv::Point2d(keypoints[1].pt.x, keypoints[1].pt.y) );    
        image_points.push_back( cv::Point2d(keypoints[2].pt.x, keypoints[2].pt.y) );    
        image_points.push_back( cv::Point2d(keypoints[3].pt.x, keypoints[3].pt.y) ); 
        image_points.push_back( cv::Point2d(keypoints[4].pt.x, keypoints[4].pt.y) ); 
    
        // 3D model points.
        std::vector<cv::Point3d> model_points;
        model_points.push_back(cv::Point3d(  0.0f,  0.0f, 0.0f));  //Center           
        model_points.push_back(cv::Point3d(  25.0f,25.0f, 0.0f));  //right top         
        model_points.push_back(cv::Point3d(-25.0f,-25.0f, 0.0f));  //left bottom    
        model_points.push_back(cv::Point3d(-25.0f, 25.0f, 0.0f));  //left top
        model_points.push_back(cv::Point3d( 25.0f,-25.0f, 0.0f));  //right bottom     
        
        
        // Camera internals
        double focal_length = frame.cols; // Approximate focal length.
        Point2d center = cv::Point2d(frame.cols/2,frame.rows/2);
        cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
        cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion
        
        cout << "Camera Matrix " << endl << camera_matrix << endl ;
        
        
        // Solve for pose
        cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
    
        
        // Project a axis onto the image plane.
        
        
        vector<Point3d> nose_end_point3D;
        vector<Point2d> nose_end_point2D;
        nose_end_point3D.push_back(Point3d(100,0,0));
        nose_end_point3D.push_back(Point3d(0,100,0));
        nose_end_point3D.push_back(Point3d(0,0,100));
        
        projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
        
        
        for(int i=0; i < image_points.size(); i++)
        {
            circle(frame, image_points[i], 3, Scalar(0,0,255), -1);
        }
        
        cv::line(frame,image_points[0], nose_end_point2D[0], cv::Scalar(255,0,0), 2);
        cv::line(frame,image_points[0], nose_end_point2D[1], cv::Scalar(0,255,0), 2);
        cv::line(frame,image_points[0], nose_end_point2D[2], cv::Scalar(0,0,255), 2);
        
        // int str = (rotation_vector.at<float>(0));
        // stringstream iss;
        // iss << str;
        // putText(frame, iss.str(), Point2d(10,20), 1, 2, Scalar(255,0,0), 2);
        cout << "Rotation Vector " << endl << rotation_vector << endl;
        cout << "Translation Vector" << endl << translation_vector << endl;
        
        cout <<  nose_end_point2D << endl;
        
        
        
        
    }
    
    // Display image.
    cv::imshow("Output", frame);

    

}    


void Detector::detectKeys(bool video, Mat frame)
{

    // show live and wait for a key with timeout long enough to show images
    imshow("Live", frame);

    // -------- gray scaling ----------

    Mat BW1;
    cvtColor(frame, BW1, CV_BGR2GRAY);
    imshow("bw1 grey", BW1);

    // -------- Masking ----------
    Mat BW2 = BW1 > bw2_Treshold;

    imshow("bw2 mask", BW2);

    // ---------Detecting keypoints ----

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;
    
    Ptr<SURF> detector = SURF::create( minHessian );
    
    std::vector<KeyPoint> keypoints_1;
    
    detector->detect( BW2, keypoints_1 );
    
    //-- Draw keypoints
    Mat img_keypoints_1;
    drawKeypoints( BW2, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    
    //-- Show detected (drawn) keypoints
    imshow("Keypoints 1", img_keypoints_1 );

}

vector<Point2d> Detector::detect(Mat frame)
{
    // show live and wait for a key with timeout long enough to show images
    imshow("Live", frame);

    // -------- gray scaling ----------

    Mat BW1;
    cvtColor(frame, BW1, CV_BGR2GRAY);
    imshow("bw1 grey", BW1);

    // -------- Masking ----------
    Mat BW2 = BW1 > bw2_Treshold;

    imshow("bw2 mask", BW2);


    //TODO: add HSV filter

    // -------- Skeleton ----------

    Mat skel(BW2.size(), CV_8UC1, Scalar(0));
    Mat temp;
    Mat eroded;
    Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
    Mat BW3 = BW2;
    bool done;
    int iterations=0;

    do
    {
    erode(BW3, eroded, element);
    dilate(eroded, temp, element);
    subtract(BW3, temp, temp);
    bitwise_or(skel, temp, skel);
    eroded.copyTo(BW3);

    done = (countNonZero(BW3) == 0);
    iterations++;
    
    } while (!done && (iterations < 100));

    //cout << "iterations=" << iterations << endl;
    
    

    //------- Dilate -------------//

    Mat BW3_5;
    dilate(skel,BW3_5,element);
    imshow("bw3: skeleton", BW3_5);
    //-------Hough lines----------//
    //C++: void HoughLines(InputArray image, OutputArray lines, double rho, double theta, int threshold, double srn=0, double stn=0 )

    Mat BW4 = frame;
    vector<Vec4i> lines;
    

    HoughLinesP( BW3_5, lines, bw4_Rho, CV_PI/bw4_Theta, bw4_Treshold, bw4_minLinLength, bw4_maxLineGap );
    /*
    vector<Vec4i> lines;
    HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
    with the arguments:

    dst: Output of the edge detector. It should be a grayscale image (although in fact it is a binary one)
    lines: A vector that will store the parameters (x_{start}, y_{start}, x_{end}, y_{end}) of the detected lines
    rho : The resolution of the parameter r in pixels. We use 1 pixel.
    theta: The resolution of the parameter \theta in radians. We use 1 degree (CV_PI/180)
    threshold: The minimum number of intersections to “detect” a line
    minLinLength: The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
    maxLineGap: The maximum gap between two points to be considered in the same line.
    
    

    */


    cout << "BW4 \t Found hough lines: \n";
    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( BW4, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 1, 8 );
        
        

        
        
        float  slope = atan( (float)( (int)lines[i][3] - (int)lines[i][1]) / (float)((int)lines[i][2] - (int)lines[i][0]) ) * (180 / 3.1415);


        cout << "BW4 \t Line: " << i << "\t P1_x, P1_y: (" << lines[i][0] << "," << lines[i][1] << ") \t P2_x, P2_y: ";
        cout << lines[i][2] << "," << lines[i][3]  << "\t Slope: " << slope << "\n";
    }

    Rect boundingBox = lines2boundingbox(frame, lines);

    //cout << "BW4 \t box: " << 1 << "\t Xmin, Xmax: (" << Xmin << "," << Xmax << ") \t Ymin,Ymax: (" << Ymin << "," << Ymax  << ")\n";
    rectangle(BW4, boundingBox, Scalar( 0, 255, 255, 125 ));
    imshow("bw4: Hough lines", BW4);
    //Mat& img, Rect rec, const Scalar& color,

    //Group overlaying lines together
    vector<Vec4i> corelines = lines2points(lines);
    


    //----- Fabriacte points for the locate alg. -----------//

    Size s = frame.size();
    // rows = s.height;
    // cols = s.width;
    vector<Point2d> output;
    vector<Point2d> centerpoints;



    Mat BW5 = frame;
    //circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)

    //Mat BW5;

    //Determinens which end points of the found core lines are in the middle of the screen and which one are on the edges
    for(size_t i = 0; i < corelines.size(); i++ )
    {
        if( ( corelines[i][0] < (s.width/2 - s.width/4) || corelines[i][0] > (s.width/2 + s.width/4) )
            ||
            ( corelines[i][1] < (s.height/2 - s.height/4) || corelines[i][1] > (s.height/2 + s.height/4) )  )
        {
            //circle(BW5,Point2d(corelines[i][0], corelines[i][1]), 5, Scalar(0,0,255),2); //red
            output.push_back(Point2d(corelines[i][0], corelines[i][1]));
        }
        else
        {
            //circle(BW5,Point2d(corelines[i][0], corelines[i][1]), 5, Scalar(255,255,0),2); //blue
            centerpoints.push_back(Point2d(corelines[i][0], corelines[i][1]));
        }

        if( ( corelines[i][2] < (s.width/2 - s.width/4) || corelines[i][2] > (s.width/2 + s.width/4) )
            || 
            (corelines[i][3] < (s.height/2 - s.height/4) || corelines[i][3] > (s.height/2 + s.height/4)))
        {
            //circle(BW5,Point2d(corelines[i][2], corelines[i][3]), 6, Scalar(0,0,255),2);
            output.push_back(Point2d(corelines[i][2], corelines[i][3]));
        }
        else
        {
            //circle(BW5,Point2d(corelines[i][2], corelines[i][3]), 6, Scalar(255,255,0),2);
            centerpoints.push_back(Point2d(corelines[i][2], corelines[i][3]));
        }
        
        


        line( BW5, Point(corelines[i][0], corelines[i][1]),
            Point(corelines[i][2], corelines[i][3]), Scalar(0,255,0), 2, 8 );
    }

    //resolve middle point of the wind turbine
    int x_acc = 0;
    int y_acc = 0;
    for(size_t i = 0; i < centerpoints.size(); i++ )
    {
        x_acc += centerpoints[i].x;
        y_acc += centerpoints[i].y;
    }

    if(centerpoints.size() != 0)
    {
        output.push_back(Point2d(x_acc / centerpoints.size(), y_acc / centerpoints.size()));

    }
    
    //debug print
    for(size_t i = 0; i < output.size(); i++ )
    {
        circle(BW5,Point2d(output[i].x, output[i].y), 6, Scalar(255,0,0),2);
    }

    imshow("Simple lines", BW5);
    return output;

}

Rect Detector::lines2boundingbox(Mat frame, vector<Vec4i> lines)
{
    int Xmax = frame.size[0]/2;
    int Xmin = frame.size[0]/2;
    int Ymax = frame.size[1]/2;
    int Ymin = frame.size[1]/2;

    Rect boundingBox;


    for( size_t i = 0; i < lines.size(); i++ )
    {

        //Create bounding box
        if (Xmax < lines[i][0] || Xmax < lines[i][2] )
        {
            if( lines[i][0] > lines[i][2])
            {
                Xmax = lines[i][0];
            }
            else
            {
                Xmax = lines[i][2];
            }
        }
        if (Xmin > lines[i][0] || Xmin > lines[i][2] )
        {
            if( lines[i][0] < lines[i][2])
            {
                Xmin = lines[i][0];
            }
            else
            {
                Xmin = lines[i][2];
            }
        }
        if (Ymax < lines[i][1] || Ymax < lines[i][3] )
        {
            if( lines[i][1] > lines[i][3])
            {
                Ymax = lines[i][1];
            }
            else
            {
                Ymax = lines[i][3];
            }
        }
        if (Ymin > lines[i][1] || Ymin > lines[i][3] )
        {
            if( lines[i][1] < lines[i][3])
            {
                Ymin = lines[i][1];
            }
            else
            {
                Ymin = lines[i][3];
            }
        }
    }

    boundingBox.x       = Xmin;
    boundingBox.y       = Ymin;
    boundingBox.width   = Xmax - Xmin; 
    boundingBox.height  = Ymax - Ymin;

    return boundingBox;
}

vector<Vec4i> Detector::lines2points( vector<Vec4i> lines )
{
    
    
    //Catogorize lines based on angle :o 
    std::vector<int> labels;
    int numberOfLines = cv::partition(lines, labels, isEqual());

    //Output container
    vector<Vec4i> lines2;

    //For all labels
    for(int i = 0; i < numberOfLines; i++)
    {
        vector<int> x;
        vector<int> y;
        bool big_x,big_y;

        //Check each line
        for(size_t j = 0; j < lines.size(); j++)
        {
            //if line has the correct label
            if(labels[j] == i )
            {
                //Save values in the containers
                x.push_back( lines[i][0]);
                x.push_back( lines[i][2]);
                y.push_back( lines[i][1]);
                y.push_back( lines[i][3]);
            }
            
            //Save whether x1 is bigger than x2. Could also be done with the slope
            if(lines[i][0] < lines[i][2])
            {
                big_x = 1;
            }
            else
            {
                big_x = 0;
            }

            if(lines[i][1] < lines[i][3])
            {
                big_y = 1;
            }
            else
            {
                big_y = 0;
            }


            

        }

        //Determine the max and minima of the array
        auto x_min_value = *std::min_element(x.begin(),x.end());
        auto x_max_value = *std::max_element(x.begin(),x.end());
        auto y_min_value = *std::min_element(y.begin(),y.end());
        auto y_max_value = *std::max_element(y.begin(),y.end());
        
        //Create output containers
        int x1,x2,y1,y2;

        //Write maxima to the output containers according the arrangement determined before
        if(big_x)
        {
            x1 = x_min_value;
            x2 = x_max_value;
        }
        else
        {
            x2 = x_min_value;
            x1 = x_max_value;
        }
        
        if(big_y)
        {
            y1 = y_min_value;
            y2 = y_max_value;
        }
        else
        {
            y2 = y_min_value;
            y1 = y_max_value;
        }

        //Add the found line the 
        lines2.push_back( Vec4i(x1, y1, x2 , y2 ) );

        

    }
    

    

    //cv::partition()
    //stoi("5");
    
    return lines2;
}
