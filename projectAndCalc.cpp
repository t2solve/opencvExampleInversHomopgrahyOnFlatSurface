//how to compile
// g++ -std=gnu++11 -o projectAndCalc projectAndCalc.cpp `pkg-config opencv --cflags --libs`
// base on realy good discussed example
// @see https://stackoverflow.com/questions/44104633/transforming-2d-image-coordinates-to-3d-world-coordinates-with-z-0
// @see https://github.com/rodolfoap/OpenCV-2Dto3D

#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <math.h> 

using namespace cv;
using namespace std;

Mat inverseHomographyMatrix;

Mat project(Mat a) 
{
    Mat point3Dw = inverseHomographyMatrix*a;
    //cout << "Point 3D-W : " << point3Dw << endl << endl;

    double w = point3Dw.at<double>(2, 0);//get W
    //cout << "W: " << w << endl << endl;

    //we need to normalise this point
    divide(point3Dw,w,point3Dw);

    return point3Dw;
} 

Mat convertPointToMat(Point2d a)
{
    return (Mat_<double>(3, 1) << a.x, a.y, 1.0);
}

Mat convertPointToMat(Point3d a)
{
    return (Mat_<double>(3, 1) << a.x, a.y, a.z);
}

double calcEuclidianDistanc(Mat a, Mat b)
{

    return cv::norm(a,b);
}

double calcEuclidianDistanceImage(Point2d a, Point2d b)
{
    Mat aI = convertPointToMat(a);
    Mat bI = convertPointToMat(b);

    return calcEuclidianDistanc(aI,bI);
}

double projectToRealAndCalcEuclidianDistance(Point2d a, Point2d b)
{
    //calc point to mat
    Mat aI = convertPointToMat(a);
    Mat bI = convertPointToMat(b);

    Mat aR = project(aI);
    Mat bR = project(bI);

    return calcEuclidianDistanc(aR,bR);
}

Mat loadFileAndCalcInverseHomographyMatrix(String filename)
{
    Mat cameraMatrix, distCoeffs, rotationVector, rotationMatrix,
    translationVector,extrinsicMatrix, projectionMatrix, homographyMatrix,
    inverseHomographyMatrix,combindedBigMat;

    FileStorage fs(filename, FileStorage::READ);

    fs["camera_matrix"] >> cameraMatrix;
    //cout << "Camera Matrix: " << cameraMatrix << endl << endl;

    fs["distortion_coefficients"] >> distCoeffs;
    //cout << "Distortion Coefficients: " << distCoeffs << endl << endl;

    //we also grep all rvec and tvec
    fs["extrinsic_parameters"] >>combindedBigMat;

    //we read the first translation and rotationVector vector
    int i=0;//for the first picture //@see camera_calbration function  saveCameraParams for read in
    Mat r = combindedBigMat(Range(int(i), int(i+1)), Range(0,3));
    Mat t = combindedBigMat(Range(int(i), int(i+1)), Range(3,6));

    //cout <<"rvec:"<<r<<endl << endl;
    //cout <<"tvec:"<<t<<endl << endl;

    rotationVector = r.t(); //we use the one from the camer calibration
    translationVector = t.t(); //we use the one from the camer calibration

    //cout << "Rotation Vector: update " << endl << rotationVector << endl << endl;
    //cout << "Translation Vector: update " << endl << translationVector << endl << endl;

    Rodrigues(rotationVector, rotationMatrix);
    //cout << "Rotation Matrix: " << endl << rotationMatrix << endl << endl;

    hconcat(rotationMatrix, translationVector, extrinsicMatrix);
    //cout << "Extrinsic Matrix: " << endl << extrinsicMatrix << endl << endl;

    projectionMatrix = cameraMatrix * extrinsicMatrix;
    //cout << "Projection Matrix: " << endl << projectionMatrix << endl << endl;

    //we get single elements
    double p11 = projectionMatrix.at<double>(0, 0),
            p12 = projectionMatrix.at<double>(0, 1),
            p14 = projectionMatrix.at<double>(0, 3),
            p21 = projectionMatrix.at<double>(1, 0),
            p22 = projectionMatrix.at<double>(1, 1),
            p24 = projectionMatrix.at<double>(1, 3),
            p31 = projectionMatrix.at<double>(2, 0),
            p32 = projectionMatrix.at<double>(2, 1),
            p34 = projectionMatrix.at<double>(2, 3);


    homographyMatrix = (Mat_<double>(3, 3) << p11, p12, p14, p21, p22, p24, p31, p32, p34);
    //cout << "Homography Matrix: " << endl << homographyMatrix << endl << endl;

    inverseHomographyMatrix = homographyMatrix.inv();


    return inverseHomographyMatrix;
}

int main()
{

    //we write the global one
    inverseHomographyMatrix = loadFileAndCalcInverseHomographyMatrix( "camera_param_d4_n500.xml");

    //! we also calc a bunch of distances for double checkinig
    //we try to load the picture and find all chessboard corners
    cv::Mat img_input = imread("cali_pic_2_d4_2018-06-28.jpg", CV_LOAD_IMAGE_COLOR);

    vector<Point2d> pointsImage;
    vector<Point3d> pointsWorld;
    //we try to find
    Size patternsize(7,7); //interior number of corners
    vector<Point2f> corners; //this will be filled by the detected corners
    int cornerAmount = 49;

    if(img_input.data )
    {

        //convert to greyscale
        //method to threshold important changes
        int binaryThreshold = 80; //TODO as parameter
        Mat imgBinary = img_input > binaryThreshold;

        //CALIB_CB_FAST_CHECK saves a lot of time on images
        //that do not contain any chessboard corners
        bool patternfound = findChessboardCorners(imgBinary, patternsize, corners,
                                                  CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                                                  + CALIB_CB_FAST_CHECK);

        //we should apply Refines the corner locations method
        //increases accurary
        Mat viewGray;
        cvtColor(img_input, viewGray, COLOR_BGR2GRAY);
        cornerSubPix( viewGray, corners, Size(11,11),Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));

        if(patternfound)
        {
            cout <<" found chess pattern: "<<endl;
            int lineX = 0;
            int lineY = 0;
            float distanceWorld = 25;

            for(size_t j=0;j<cornerAmount;j++)
            {
                Point2f p =  corners[j];
                cout <<"["<<lineX<<"]["<<lineY<<"] - j:\t"<< j<<"  corners:\t"<<p.x<<" - \t"<<p.y<<endl;
                //circle(imgFoo , p, 10, colorRed );

                pointsImage.push_back(p);
                //we construct the real world chessboard cluster
                pointsWorld.push_back(Point3d(lineX*distanceWorld,lineY*distanceWorld,0));

                //set new line x and line y
                lineX++;
                if(lineX>=7)
                {    lineX = 0;
                    lineY++;
                }
            }

            //cv::imwrite("chessPointsDBG.jpg", imgFoo);
        }
        //drawChessboardCorners(imgBinary, patternsize, Mat(corners), patternfound);

        //cv::imwrite("chessfindDBG.jpg", imgBinary);
    }

    //! we calc the distance from projected points
    int lineX = 0;
    int lineY = 0;
    double disErr = 0.0;
    double n = 0.0;

    for(int index =1;index<cornerAmount;index++)
    {
        Point2d pIa =  pointsImage[index-1];
        Point2d pIb =  pointsImage[index];

                //set new line x and line y
                lineX++;
        if(lineX>=7)
        {    lineX = 0;
            lineY++;
        }
        else
        {
            //we calc distance and dry to
             double distanceRealInMM = projectToRealAndCalcEuclidianDistance(pIa,pIb);

             cout<<index << " compare: "<< distanceRealInMM << " to 25 mm"<<endl;

             disErr+= abs(distanceRealInMM-25.0);
             n++;
        }

    }
    disErr = disErr/n;
    cout<<"avg reprojection error: "<<disErr <<"mm for "<< n<<" measurements"<<endl;


    return 0;
}
