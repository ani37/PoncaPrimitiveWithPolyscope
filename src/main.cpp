#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>
#include <stdio.h>
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
    enum
    {
        Dim = DIMENSION
    };
    typedef double Scalar;
    typedef Eigen::Matrix<Scalar, Dim, 1> VectorType;
    typedef Eigen::Matrix<Scalar, Dim, Dim> MatrixType;

    PONCA_MULTIARCH inline MyPoint(const std::array<Scalar, 3> &poss,
                                   const std::array<Scalar, 3> &norm)
        : m_pos(Eigen::Map<const VectorType>(poss.begin())),
          m_normal(Eigen::Map<const VectorType>(norm.begin()))
    {
    }

    PONCA_MULTIARCH inline const Eigen::Map<const VectorType> &pos() const { return m_pos; }
    PONCA_MULTIARCH inline const Eigen::Map<const VectorType> &normal() const { return m_normal; }

private:
    Eigen::Map<const VectorType> m_pos, m_normal;
};

typedef MyPoint::Scalar Scalar;
typedef MyPoint::VectorType VectorType;


// Define related structure
typedef DistWeightFunc<MyPoint, SmoothWeightKernel<Scalar>> WeightFunc;

void loadPointCloud(std::string filename,
                    std::vector<std::array<double, 3>> &vertexPositionsOut)
{

    happly::PLYData plyIn(filename);

    // Get mesh-style data from the object
    vertexPositionsOut = plyIn.getVertexPositions();
    //faceIndicesOut = plyIn.getFaceIndices<size_t>();
}

int knei = 10;
Scalar tmax = 0.1;

// to compute normals or curvature generalised using template and functors using KNN
template <typename FitT>
vector<VectorType> compute(const vector<MyPoint>& points)
{
    KdTree<MyPoint> kdtree(points);
    vector<VectorType> projection(points.size());
    for (int i = 0; i < points.size(); i++)
    {
        // set evaluation point and scale at the ith coordinate
        const VectorType &p = points.at(i).pos();
        // Here we now perform the fit
        FitT _fit;
        // Set a weighting function instance
        _fit.setWeightFunc(WeightFunc(tmax));
        // Set the evaluation position
        _fit.init(p);
        for (auto idx : kdtree.k_nearest_neighbors(p, knei))
        {
            _fit.addNeighbor(points[idx]);
        }
        _fit.finalize();
        if(_fit.isStable())
        {
            
            projection[i] = _fit.project(p).transpose();
        }
    }

    return projection;
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
            typedef Basket<MyPoint, WeightFunc, LeastSquareLine> Linefit;
            
            string filename = "line.ply";
            std::ifstream testStream(filename);
            if (!testStream)
            {
                exit(0);
            }

            std::vector<std::array<double, 3>> positions;
            /* Load positions from file */
            loadPointCloud(filename, positions);
            testStream.close();
            
            /* visualize! */
            polyscope::registerPointCloud("Line positions", positions);
            ImGui::InputInt("Variable K", &knei);          // set a float variable

            if (ImGui::Button("Find Projections"))
            {
                vector<MyPoint> points;
                for (const auto &p : positions)
                {
                    points.push_back({p, {0, 0, 0}});
                }

                vector<VectorType> projected(compute<Linefit>(points));
                polyscope::registerPointCloud("Line projections", projected);

            }
            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Plane Fitting"))
        {
            typedef Basket<MyPoint, WeightFunc, CovariancePlaneFit> PlaneFit;
        
            string filename = "hippo.ply";
            std::ifstream testStream(filename);
            if (!testStream)
            {
                exit(0);
            }

            std::vector<std::array<double, 3>> positions;
            /* Load positions from file */
            loadPointCloud(filename, positions);
            testStream.close();
            
            /* visualize! */
            polyscope::registerPointCloud("Plane positions", positions);
            ImGui::InputInt("Variable K", &knei);          // set a float variable

            if (ImGui::Button("Find Projections"))
            {
                vector<MyPoint> points;
                for (const auto &p : positions)
                {
                    points.push_back({p, {0, 0, 0}});
                }

                vector<VectorType> projected(compute<PlaneFit>(points));
                polyscope::registerPointCloud("Plane projections", projected);
            }
            ImGui::TreePop();
        }


        if (ImGui::TreeNode("Sphere Fitting"))
        {
            typedef Basket<MyPoint, WeightFunc, OrientedSphereFit> SphereFit;
            string filename = "hippo.ply";
            std::ifstream testStream(filename);
            if (!testStream)
            {
                exit(0);
            }

            std::vector<std::array<double, 3>> positions;
            /* Load positions from file */
            loadPointCloud(filename, positions);
            testStream.close();
            
            /* visualize! */
            polyscope::registerPointCloud("Sphere positions", positions);
            ImGui::InputInt("Variable K", &knei);          // set a float variable

            if (ImGui::Button("Find Projections"))
            {
                vector<MyPoint> points;
                for (const auto &p : positions)
                {
                    points.push_back({p, {0, 0, 0}});
                }

                vector<VectorType> projected(compute<SphereFit>(points));
                polyscope::registerPointCloud("Sphere projections", projected);
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
    // Show the gui
    polyscope::show();
    return 0;
}
