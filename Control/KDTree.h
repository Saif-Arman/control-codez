#pragma once
#include <queue>
#include <vector>
#include "Macro.h"

class KDTree
{
public:
    KDTree();

    ~KDTree();

    // Define a point in 3D space (yaw, pitch, roll)
    struct Point
    {
        double yaw;
        double pitch;
        double roll;
        double yaw_vec;
        double pitch_vec;
        double roll_vec;
        double fx;
        double fy;
        double fz;
        double tx;
        double ty;
        double tz;

        Point(double y, double p, double r, double y_v, double p_v, double r_v, double fx, double fy, double fz, double tx, double ty, double tz)
            : yaw(y)
            , pitch(p)
            , roll(r)
            , yaw_vec(y_v)
            , pitch_vec(p_v)
            , roll_vec(r_v)
            , fx(fx)
            , fy(fy)
            , fz(fz)
            , tx(tx)
            , ty(ty)
            , tz(tz)
        {
        }

        Point(double y, double p, double r)
            : yaw(y)
            , pitch(p)
            , roll(r)
            , yaw_vec(0)
            , pitch_vec(0)
            , roll_vec(0)
            , fx(0)
            , fy(0)
            , fz(0)
            , tx(0)
            , ty(0)
            , tz(0)
        {
        }
    };

private:
    // KD-tree node
    struct KDNode
    {
        Point point;
        KDNode* left;
        KDNode* right;

        KDNode(const Point& pt) : point(pt), left(nullptr), right(nullptr) {}
    };

    // Function to build the KD-tree
    KDNode* buildKDTree(std::vector<Point>::iterator begin, std::vector<Point>::iterator end, int depth);

    // Function to calculate squared Euclidean distance between two points
    double squaredDistance(const Point& a, const Point& b);

    // Custom comparator for the priority queue
    struct CompareDist
    {
        bool operator()(const std::pair<double, Point>& a, const std::pair<double, Point>& b) const
        {
            return a.first < b.first; // For max heap: largest distance has higher priority
        }
    };

    // Function to delete the KD-tree and free memory
    void deleteKDTree(KDNode* node);

    void readPoints(std::vector<double>& points, std::vector<Point>& pointList);

    std::array<double, 3> anglesToUnitVector(double yaw, double pitch, double roll);

    // Function to read yaw, pitch, and roll from a CSV file, skipping the first line
    void readYawPitchRollFromCSV(const std::string& filename, std::vector<Point>& points);

    std::string _calFile;

    KDNode* _root;

public:

    // Function to find the k nearest neighbors
    void kNearestNeighbors(KDNode* root, const Point& target, int k, int depth,
        std::priority_queue<std::pair<double, Point>,
        std::vector<std::pair<double, Point>>,
        CompareDist>& maxHeap);

    /*inline void setPointFileName(std::string filename) { _calFile = filename; };*/
    
    void initialize(std::string calibration_pt_file);

    std::array<double, 6> get_ft_offset(double yaw, double pitch, double roll);
};

