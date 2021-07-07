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

using namespace std;
using namespace Ponca;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define DIMENSION 3
 
/*
   \brief Variant of the MyPoint class allowing to work with external raw data.
 
   Using this approach, ones can use the patate library with already existing
   data-structures and without any data-duplication.
 
   In this example, we use this class to find Normals and curvature  from a polygon file using KdTree.
 */
// This class defines the input data format
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
 
vector<MyPoint> points;



// Define related structure
typedef DistWeightFunc<MyPoint,SmoothWeightKernel<Scalar> > WeightFunc;

void loadPointCloud(std::string filename,
                    std::vector<std::array<double, 3>>& vertexPositionsOut) {

  happly::PLYData plyIn(filename);

  // Get mesh-style data from the object
  vertexPositionsOut = plyIn.getVertexPositions();
  //faceIndicesOut = plyIn.getFaceIndices<size_t>();

}


int knei = 10;
Scalar tmax = 0.1;

// to compute normals generalised using template and functors using KNN
template<typename FitT, typename Functor>
void computeKnn(const KdTree<MyPoint>& structure, Functor f)
{


    for(int i = 0; i < points.size(); i++){
        // set evaluation point and scale at the ith coordinate
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

// to compute normals generalised using template and functors using Range Neighbors
template<typename FitT, typename Functor>
void computeRangeNeighbors(const KdTree<MyPoint>& structure, Functor f)
{

    
    for(int i = 0; i < points.size(); i++){
        // set evaluation point and scale at the ith coordinate
        const VectorType& p = points.at(i).pos();
        // Here we now perform the fit
        FitT _fit;
        // Set a weighting function instance
        _fit.setWeightFunc(WeightFunc(tmax));
        // Set the evaluation position
        _fit.init(p);
        for( auto idx : structure.range_neighbors(p, tmax) ){
            _fit.addNeighbor( points[idx] );
        }
        _fit.finalize();
	     f( i, _fit );
    }
}


// Your callback functions
void myCallback() {

    // Since options::openImGuiWindowForUserCallback == true by default, 
    // we can immediately start using ImGui commands to build a UI

    ImGui::PushItemWidth(100); // Make ui elements 100 pixels wide,
                                // instead of full width. Must have 
                                // matching PopItemWidth() below.
                            


    KdTree<MyPoint> kdtree(points);


    if (ImGui::TreeNode("Fitting"))
    {
        if (ImGui::TreeNode("Line Fitting"))
        {

           if(ImGui::Button("Project points on the line"))
           {

           }
           ImGui::TreePop();
        }

        if (ImGui::TreeNode("Plane Fitting"))
        {
            
           
            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Sphere Fitting"))
        {


            string filename = "hippo.ply";

            std::ifstream testStream(filename);
            if (!testStream) {
                exit(0);
            }

            std::vector< std::array<double, 3> > positions;
            // Load positions from file
            loadPointCloud(filename, positions);
            testStream.close();

            int n = positions.size();

            

            for(int i = 0; i < n; i++){
                points.push_back ( {positions[i], {0,0,0}} );
            }
        

            // visualize!
            polyscope::registerPointCloud("SphereFittingPositions", positions);

            ImGui::InputDouble("Scalar attribute", &tmax);  // set a double variable
            ImGui::InputInt("Variable K", &knei);  // set a float variable
            // KNN

            if(ImGui::Button("Compute curvature with knn"))
            {

                typedef Basket<MyPoint,WeightFunc,OrientedSphereFit,   GLSParam> SphereFit;

                std::vector< Scalar > curvature(points.size());  

                computeKnn<SphereFit>( kdtree, [&curvature]( int i, const SphereFit& _fit ) // c++lambda
                {
                    if( _fit.isStable() ){
                        curvature[i] = _fit.kappa();  

                    } 
                    else
                        curvature[i] = 0;  
                }
                );
                polyscope::getPointCloud("positions")->addScalarQuantity("curvature_knn", curvature);
            }

            

            if (ImGui::Button("Compute Normals with knn")) 
            {
                typedef Basket<MyPoint,WeightFunc,CovariancePlaneFit> PlaneFit;

                std::vector< std::array<double, 3> > normals(points.size());  

                computeKnn<PlaneFit>( kdtree, [&normals]( int i, const PlaneFit& _fit ) // c++lambda
                {
                    if( _fit.isStable() ){
                            VectorType no = _fit.primitiveGradient(points.at(i).pos());
                            normals[i] = {no[0],no[1], no[2]};
                        } else
                        normals[i] = {0,0,0};
                }
                );
                polyscope::getPointCloud("positions")->addVectorQuantity("normals_knn", normals);
            } 


            // RN

            if(ImGui::Button("Compute curvature with RN")) 
            {

                typedef Basket<MyPoint,WeightFunc,OrientedSphereFit,   GLSParam> SphereFit;

                std::vector< Scalar > curvature(points.size());  

                computeRangeNeighbors<SphereFit>( kdtree, [&curvature]( int i, const SphereFit& _fit ) // c++lambda
                {
                    if( _fit.isStable() ){
                        curvature[i] = _fit.kappa();  

                    } 
                    else
                        curvature[i] = 0;  
                }
                );
                polyscope::getPointCloud("positions")->addScalarQuantity("curvature_RN", curvature);
            }

            

            if (ImGui::Button("Compute Normals with RN"))
            {
                typedef Basket<MyPoint,WeightFunc,CovariancePlaneFit> PlaneFit;

                std::vector< std::array<double, 3> > normals(points.size());  

                computeRangeNeighbors<PlaneFit>( kdtree, [&normals]( int i, const PlaneFit& _fit ) // c++lambda
                {
                    if( _fit.isStable() ){
                            VectorType no = _fit.primitiveGradient(points.at(i).pos());
                            normals[i] = {no[0],no[1], no[2]};
                        } else
                        normals[i] = {0,0,0};
                }
                );

                polyscope::getPointCloud("positions")->addVectorQuantity("normals_RN", normals);
            } 

        

                    
                
                    ImGui::TreePop();
                }

        ImGui::TreePop();
    }

    ImGui::PopItemWidth();
}




int main(int argc, char **argv) {

   
    polyscope::init();
    
    // Add the callback  
    polyscope::state::userCallback = myCallback;
    // Show the gui
    polyscope::show(); 
    return 0;   
}
