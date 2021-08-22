
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
<<<<<<< HEAD
    enum {Dim = DIMENSION};
=======
    enum
    {
        Dim = DIMENSION
    };

>>>>>>> b40f58355bce490ea23ae6fb6e4e97a8a1b2fe35
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

<<<<<<< HEAD
=======
    //checking the file
    std::ifstream testStream(filename);
    if (!testStream)
    {
        exit(0);
    }
    std::vector<std::array<double, 3>> positions;
    /* Load positions from file */
    loadPointCloud(filename, positions);
    testStream.close();
    string type = typeid(FitT).name();
    type = type.substr(80);
    type = type.substr(0,type.find("ENS"));
    
    /* visualize! */
    polyscope::registerPointCloud(type + " positions", positions);

    vector<MyPoint> points;

    /*
     Note : In case of Sphere fitting if first if there are no normals present,
      we will first need to compute normals using plane fitting and then show projection
    */

    for (const auto &p : positions)
    {
        points.push_back({p, {0, 0, 0}});
    }
>>>>>>> b40f58355bce490ea23ae6fb6e4e97a8a1b2fe35



int knei = 50;
Scalar tmax = 5;
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
<<<<<<< HEAD
}

 
int main(int argc, char **argv) {

   
    polyscope::init();
=======
    /* visualize! */
    polyscope::registerPointCloud(type + " projections", projection);

}

// Your callback functions
void myCallback()
{

    // Since options::openImGuiWindowForUserCallback == true by default,
    // we can immediately start using ImGui commands to build a UI

    ImGui::PushItemWidth(100); // Make ui elements 100 pixels wide,
                               // instead of full width. Must have
                               // matching PopItemWidth() below.
    if (ImGui::TreeNode("Fitting"))
    {
        if (ImGui::TreeNode("Line Fitting"))
        {
            string filename = "line.ply";
            ImGui::InputInt("Variable K", &knei);          // set a float variable
            ImGui::InputDouble("Scalar attribute", &tmax);  // set a double variable

            if (ImGui::Button("Find Projections"))
            {
                typedef Basket<MyPoint, WeightFunc, CovarianceLineFit> Linefit;
                compute<Linefit>(filename);
            }
>>>>>>> b40f58355bce490ea23ae6fb6e4e97a8a1b2fe35

    string filename = "x_y_z.csv";
   
    std::vector< std::array<Scalar, 3> > positions(read_csv(filename));    

<<<<<<< HEAD
    int n = 1000;
    std::vector< std::array<Scalar, 3> > pro(n); 
=======
        if (ImGui::TreeNode("Plane Fitting"))
        {
            string filename = "hippo.ply";
            ImGui::InputInt("Variable K", &knei);          // set a float variable
            ImGui::InputDouble("Scalar attribute", &tmax);  // set a double variable
>>>>>>> b40f58355bce490ea23ae6fb6e4e97a8a1b2fe35

    for(int i = 0; i < n; i++){
         VectorType a = VectorType::Random();
  
         a[2] = -(a[0] + a[1]*a[1]) ;
         pro[i] = {a[0], a[1], a[2]};
     }

    for(int i = 0; i < n; i++){
        points.push_back({pro[i], {0,0,0}});
    }

<<<<<<< HEAD
    KdTree<MyPoint> kdtree(points);
     
    std::vector< VectorType > projected(points.size()); 

    computeKnn<Surfacefit>(points, kdtree, [&projected]( int i,  Surfacefit& _fit ) // c++lambda
    {
        if(_fit.isStable() )
=======
        if (ImGui::TreeNode("Sphere Fitting"))
        {
            string filename = "hippo.ply";
            ImGui::InputInt("Variable K", &knei);          // set a float variable
            ImGui::InputDouble("Scalar attribute", &tmax);  // set a double variable
            
            if (ImGui::Button("Find Projections"))
>>>>>>> b40f58355bce490ea23ae6fb6e4e97a8a1b2fe35
            {
                
                projected[i] = _fit.project(points[i].pos());
                cout <<  _fit.getParameters()  << "\n\n\n";
            } 
            else
            projected[i] = {0,0,0};  
    });

<<<<<<< HEAD
    // // visualize!
    polyscope::registerPointCloud("positions", pro);
    polyscope::registerPointCloud("projection", projected);
    //polyscope::getPointCloud("positions")->addVectorQuantity("normals", projected);


=======
            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Surface Fitting"))
        {
            string filename = "hippo.ply";
            ImGui::InputInt("Variable K", &knei);          // set a float variable
            ImGui::InputDouble("Scalar attribute", &tmax);  // set a double variable
            
            if (ImGui::Button("Find Projections"))
            {
                typedef Basket<MyPoint, WeightFunc, LeastSquareSurfaceFit> Surface;
                compute<Surface>(filename);
            }

            ImGui::TreePop();
        }

        ImGui::TreePop();
    }

    ImGui::PopItemWidth();
}

int main(int argc, char **argv)
{
    polyscope::init();
    // Add the callback
    polyscope::state::userCallback = myCallback;
>>>>>>> b40f58355bce490ea23ae6fb6e4e97a8a1b2fe35
    // Show the gui
    polyscope::show(); 
    return 0;   
}
