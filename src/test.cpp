
#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>
#include<stdio.h>
#include "happly.h"
#include <random>
#include <string>
#include <iterator>
//Ponca
#include <Ponca/Fitting>
#include "Eigen/Eigen"
#include <Ponca/src/SpatialPartitioning/KdTree/kdTree.h>


// Polyscope
#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "polyscope/curve_network.h"

#include <Eigen/Core> 
#include <Eigen/Dense>
using namespace Eigen;

using namespace std;
using namespace Ponca;


#define DIMENSION 3
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 

class MyPoint
{
public:
    enum {Dim = DIMENSION};
    typedef double Scalar;
    typedef Eigen::Matrix<Scalar, Dim, 1>   VectorType;
    typedef Eigen::Matrix<Scalar, Dim, Dim> MatrixType;
 
    PONCA_MULTIARCH inline MyPoint(const std::array<Scalar, 3>&poss,
                                   const std::array<Scalar, 3>&  norm)
        : m_pos    (Eigen::Map< const VectorType >(poss.begin())),
          m_normal (Eigen::Map< const VectorType >(norm.begin()))
    {}
 
    PONCA_MULTIARCH inline const Eigen::Map< const VectorType >& pos()    const { return m_pos; }
    PONCA_MULTIARCH inline const Eigen::Map< const VectorType >& normal() const { return m_normal; }
 
private:
    Eigen::Map< const VectorType > m_pos, m_normal;
}; 


typedef MyPoint::Scalar Scalar;
typedef MyPoint::VectorType VectorType;
// Define related structure
typedef DistWeightFunc<MyPoint,SmoothWeightKernel<Scalar> > WeightFunc;
typedef Basket<MyPoint,WeightFunc, LeastSquareSurfaceFit> Surfacefit;


vector<string> split(string str, char dl)
{
    string word = "";
 
    // to count the number of split strings
    int num = 0;
 
    // adding delimiter character at the end
    // of 'str'
    str = str + dl;
 
    // length of 'str'
    int l = str.size();
 
    // traversing 'str' from left to right
    vector<string> substr_list;
    for (int i = 0; i < l; i++) {
 
        // if str[i] is not equal to the delimiter
        // character then accumulate it to 'word'
        if (str[i] != dl)
            word = word + str[i];
 
        else {
 
            // if 'word' is not an empty string,
            // then add this 'word' to the array
            // 'substr_list[]'
            if ((int)word.size() != 0)
                substr_list.push_back(word);
 
            // reset 'word'
            word = "";
        }
    }
 
    // return the splitted strings
    return substr_list;
}
void loadPointCloud(std::string filename,
                    std::vector<std::array<double, 3>>& vertexPositionsOut) {

  happly::PLYData plyIn(filename);

  // Get mesh-style data from the object
  vertexPositionsOut = plyIn.getVertexPositions();
  //faceIndicesOut = plyIn.getFaceIndices<size_t>();


}
std::vector<std::array<Scalar, 3> > read_csv(string filename)
{
  char delimiter = ',';
  std::vector<std::array<Scalar, 3> > point_cloud;
 
  //readfile
  fstream file;
  file.open(filename);
  std::string line;
  while (getline( file, line,'\n'))  
	{
     
	  istringstream templine(line); 
	  string data;

    vector<string> v = split (line, delimiter);
    std::array<Scalar, 3> d({stod(v[1]),stod(v[2]),stod(v[3])});
    point_cloud.push_back(d);

	}
  file.close();
  return point_cloud;
}




int knei = 50;
Scalar tmax = 10;
vector<MyPoint> points;

// to compute normals generalised using template and functors using KNN
template<typename FitT, typename Functor>
void computeKnn(const vector<MyPoint> points, const KdTree<MyPoint>& structure, Functor f)
{

    for(int i = 0; i < points.size(); i++){
        // set evaluation 
       // point and scale at the ith coordinate
        const VectorType& p = points.at(i).pos();
        // Here we now perform the fit
        FitT _fit;
        // Set a weighting function instance
        _fit.setWeightFunc(WeightFunc(tmax));
        // Set the evaluation position
        _fit.init(p);
        for( auto idx : structure.k_nearest_neighbors(p, knei) ){
            _fit.addNeighbor( points[idx] );
        }

        _fit.finalize();
	     f( i, _fit );
    }
}

 
int main(int argc, char **argv) {

   
    polyscope::init();

    string filename = "x_y_z.csv";
   
    std::vector< std::array<Scalar, 3> > positions(read_csv(filename));    

    int n = 1000;
    std::vector< std::array<Scalar, 3> > pro(n); 

    for(int i = 0; i < n; i++){
         VectorType a = VectorType::Random();
  
         a[2] = -(a[0] + a[1]*a[1] ) ;
         pro[i] = {a[0], a[1], a[2]};
     }

    for(int i = 0; i < n; i++){
        points.push_back({pro[i], {0,0,0}});
    }

    KdTree<MyPoint> kdtree(points);
     
    std::vector< VectorType > projected(points.size()); 

    computeKnn<Surfacefit>(points, kdtree, [&projected]( int i,  Surfacefit& _fit ) // c++lambda
    {
        if(_fit.isStable() )
            {
                
                projected[i] = _fit.project(points[i].pos());
                cout <<  _fit.getParameters()  << "\n\n\n";
            } 
            else
            projected[i] = {0,0,0};  
    });

    // // visualize!
    polyscope::registerPointCloud("positions", pro);
    polyscope::registerPointCloud("projection", projected);
    //polyscope::getPointCloud("positions")->addVectorQuantity("normals", projected);


    // Show the gui
    polyscope::show(); 
    return 0;   
}
