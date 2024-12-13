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
        double yaw_r; // yaw in radians
        double pitch_r; // pitch in radians
        double roll_r; // roll in radians
        double yaw_d; // yaw in degrees
        double pitch_d; // pitch in degrees
        double roll_d; // roll in degrees
        double fx;
        double fy;
        double fz;
        double tx;
        double ty;
        double tz;

        Point(double y, double p, double r, double y_v, double p_v, double r_v, double fx, double fy, double fz, double tx, double ty, double tz)
            : yaw_r(y)
            , pitch_r(p)
            , roll_r(r)
            , yaw_d(y_v)
            , pitch_d(p_v)
            , roll_d(r_v)
            , fx(fx)
            , fy(fy)
            , fz(fz)
            , tx(tx)
            , ty(ty)
            , tz(tz)
        {
        }

        Point(double y, double p, double r)
            : yaw_r(y)
            , pitch_r(p)
            , roll_r(r)
            , yaw_d(0)
            , pitch_d(0)
            , roll_d(0)
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
    KDNode* build_KD_tree(std::vector<Point>::iterator begin, std::vector<Point>::iterator end, int depth);

    // Function to calculate squared Euclidean distance between two points
    double squared_distance(const Point& a, const Point& b);
    double squared_distance_ft(const KDTree::Point& a, const KDTree::Point& b);

    // Custom comparator for the priority queue
    struct CompareDist
    {
        bool operator()(const std::pair<double, Point>& a, const std::pair<double, Point>& b) const
        {
            return a.first < b.first; // For max heap: largest distance has higher priority
        }
    };

    // Function to delete the KD-tree and free memory
    void delete_KD_tree(KDNode* node);

    void read_points(std::vector<double>& points, std::vector<Point>& pointList);

    std::array<double, 3> angles_to_radians(double yaw, double pitch, double roll);

    // Function to read yaw, pitch, and roll from a CSV file, skipping the first line
    void read_yawpitchroll_from_csv(const std::string& filename, std::vector<Point>& points);

    std::string _calFile;

    KDNode* _root;

public:

    // Function to find the k nearest neighbors
    void kNearestNeighbors(KDNode* root, const Point& target, int k, int depth,
        std::priority_queue<std::pair<double, Point>,
        std::vector<std::pair<double, Point>>,
        CompareDist>& maxHeap);

    void KDTree::kNearestNeighbors_ypr(KDNode* root, const KDTree::Point& target, int k, int depth,
        std::priority_queue<std::pair<double, KDTree::Point>,
        std::vector<std::pair<double, KDTree::Point>>,
        CompareDist>& maxHeap);

    std::array<double, 3> get_ypr_offsets(double yaw, double pitch, double roll, std::array<double, 6>& ft_vector);

    /*inline void setPointFileName(std::string filename) { _calFile = filename; };*/
    
    void initialize(std::string _calibration_pt_file);

    std::array<double, 6> get_ft_offset(double yaw, double pitch, double roll);
};

