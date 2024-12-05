#include <iostream>
#include <queue>
#include <fstream>
#include <sstream>
#include <array>

#include "KDTree.h"
#include "Macro.h"
#include "ForceTorqueManager.h"

KDTree::KDTree()
{
    // Nothing to do
}

KDTree::~KDTree()
{
    // Cleanup
    deleteKDTree(_root);
}

// Function to build the KD-tree
KDTree::KDNode* KDTree::buildKDTree(std::vector<KDTree::Point>::iterator begin, std::vector<KDTree::Point>::iterator end, int depth)
{
    if (begin >= end)
    {
        return nullptr;
    }

    // Select axis based on depth so that axis cycles through all valid values
    int axis = depth % 3;
    auto comparator = [axis](const KDTree::Point& a, const KDTree::Point& b) -> bool
    {
        if (axis == 0)
            return a.yaw_r < b.yaw_r;
        else if (axis == 1)
            return a.pitch_r < b.pitch_r;
        else
            return a.roll_r < b.roll_r;
    };

    // Sort point list and choose median as pivot element
    auto mid = begin + (end - begin) / 2;
    std::nth_element(begin, mid, end, comparator);

    // Create node and construct subtrees
    KDNode* node = new KDNode(*mid);
    node->left = buildKDTree(begin, mid, depth + 1);
    node->right = buildKDTree(mid + 1, end, depth + 1);

    return node;
}

// Function to calculate squared Euclidean distance between two points
double KDTree::squaredDistance(const KDTree::Point& a, const KDTree::Point& b)
{
    return (a.yaw_r - b.yaw_r) * (a.yaw_r - b.yaw_r) +
        (a.pitch_r - b.pitch_r) * (a.pitch_r - b.pitch_r) +
        (a.roll_r - b.roll_r) * (a.roll_r - b.roll_r);
}

double KDTree::squaredDistance_ft(const KDTree::Point& a, const KDTree::Point& b)
{
    return (a.fx - b.fx) * (a.fx - b.fx) +
        (a.fy - b.fy) * (a.fy - b.fy) +
        (a.fz - b.fz) * (a.fz - b.fz);
}

// Function to find the k nearest neighbors
void KDTree::kNearestNeighbors(KDNode* root, const KDTree::Point& target, int k, int depth,
    std::priority_queue<std::pair<double, KDTree::Point>,
    std::vector<std::pair<double, KDTree::Point>>,
    CompareDist>& maxHeap)
{
    if (!root)
        return;

    // Compute squared distance between target and current point
    double dist = squaredDistance(root->point, target);

    // If heap is not full, push current point
    if (static_cast<int>(maxHeap.size()) < k)
    {
        //maxHeap.emplace(dist, root->point);
        maxHeap.push(std::make_pair(dist, root->point));
    }
    // If current point is closer than the farthest point in heap, replace it
    else if (dist < maxHeap.top().first)
    {
        maxHeap.pop();
        //maxHeap.emplace(dist, root->point);
        maxHeap.push(std::make_pair(dist, root->point));
    }

    // Determine which side to search first
    int axis = depth % 3;
    double diff;
    if (axis == 0)
        diff = target.yaw_r - root->point.yaw_r;
    else if (axis == 1)
        diff = target.pitch_r - root->point.pitch_r;
    else
        diff = target.roll_r - root->point.roll_r;

    KDNode* first = diff < 0 ? root->left : root->right;
    KDNode* second = diff < 0 ? root->right : root->left;

    // Search nearer subtree first
    KDTree::kNearestNeighbors(first, target, k, depth + 1, maxHeap);

    // Check if we need to search the farther subtree
    if (static_cast<int>(maxHeap.size()) < k || diff * diff < maxHeap.top().first)
    {
        KDTree::kNearestNeighbors(second, target, k, depth + 1, maxHeap);
    }
}

// Function to find the k nearest neighbors by force
void KDTree::kNearestNeighbors_ypr(KDNode* root, const KDTree::Point& target, int k, int depth,
    std::priority_queue<std::pair<double, KDTree::Point>,
    std::vector<std::pair<double, KDTree::Point>>,
    CompareDist>& maxHeap)
{
    if (!root)
        return;

    // Compute squared distance between target and current point
    double dist = squaredDistance_ft(root->point, target);

    // If heap is not full, push current point
    if (static_cast<int>(maxHeap.size()) < k)
    {
        //maxHeap.emplace(dist, root->point);
        maxHeap.push(std::make_pair(dist, root->point));
    }
    // If current point is closer than the farthest point in heap, replace it
    else if (dist < maxHeap.top().first)
    {
        maxHeap.pop();
        //maxHeap.emplace(dist, root->point);
        maxHeap.push(std::make_pair(dist, root->point));
    }

    // Determine which side to search first
    int axis = depth % 3;
    double diff;
    if (axis == 0)
        diff = target.fx - root->point.fx;
    else if (axis == 1)
        diff = target.fy - root->point.fy;
    else
        diff = target.fz - root->point.fz;

    KDNode* first = diff < 0 ? root->left : root->right;
    KDNode* second = diff < 0 ? root->right : root->left;

    // Search nearer subtree first
    KDTree::kNearestNeighbors_ypr(first, target, k, depth + 1, maxHeap);

    // Check if we need to search the farther subtree
    if (static_cast<int>(maxHeap.size()) < k || diff * diff < maxHeap.top().first)
    {
        KDTree::kNearestNeighbors_ypr(second, target, k, depth + 1, maxHeap);
    }
}

// Function to delete the KD-tree and free memory
void KDTree::deleteKDTree(KDTree::KDNode* node)
{
    if (node)
    {
        KDTree::deleteKDTree(node->left);
        KDTree::deleteKDTree(node->right);
        delete node;
    }
}

void KDTree::readPoints(std::vector<double>& points, std::vector<KDTree::Point>& pointList)
{
    for (int i = 0; i < points.size(); i += 3)
    {
        KDTree::Point pt(points[i], points[i + 1], points[i + 2]);
        pointList.push_back(pt);
    }
}

std::array<double, 3> KDTree::anglesToUnitVector(double yaw, double pitch, double roll)
{
    //// Convert angles from degrees to radians
    //double yawRad = yaw * M_PI / 180.0;
    //double pitchRad = pitch * M_PI / 180.0;
    //double rollRad = roll * M_PI / 180.0;

    //// Compute components based on yaw, pitch, and roll
    //double cosYaw = std::cos(yawRad);
    //double sinYaw = std::sin(yawRad);
    //double cosPitch = std::cos(pitchRad);
    //double sinPitch = std::sin(pitchRad);
    //double cosRoll = std::cos(rollRad);
    //double sinRoll = std::sin(rollRad);

    //// Apply yaw, pitch, and roll rotations to compute the unit vector components
    //double x = cosYaw * cosPitch;
    //double y = sinYaw * cosPitch;
    //double z = sinPitch;

    //// Now apply roll: roll rotates the vector around the y-axis (forward/back)
    //double newX = x * cosRoll + z * sinRoll;
    //double newZ = -x * sinRoll + z * cosRoll;

    //// Return the final unit vector as an array
    //std::array<double, 3> result = { newX, y, newZ };
    //return result;

    double yawRad = yaw * M_PI / 180.0;
    double pitchRad = pitch * M_PI / 180.0;
    double rollRad = roll * M_PI / 180.0;

    std::array<double, 3> results = { yawRad, pitchRad, rollRad };
    return results;

}

/*std::array<double, 3> KDTree::unitVectorToAngles(double x, double y, double z)
{
    // Convert angles from degrees to radians
    double yawRad = yaw * M_PI / 180.0;
    double pitchRad = pitch * M_PI / 180.0;
    double rollRad = roll * M_PI / 180.0;

    // Compute unit vector components
    double x = std::cos(yawRad) * std::cos(pitchRad);
    double y = std::sin(yawRad) * std::cos(pitchRad);
    double z = std::sin(pitchRad);

    double yawrad = std::sqrt(x);

    std::array<double, 3> result = { yaw, pitch, roll };
    return result;
}*/

// Function to read yaw, pitch, and roll from a CSV file, skipping the first line
void KDTree::readYawPitchRollFromCSV(const std::string& filename, std::vector<KDTree::Point>& points)
{
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    std::string line;

    // Skip the first line (header)
    if (!std::getline(file, line))
    {
        std::cerr << "Error reading the header line." << std::endl;
        file.close();
        return;
    }

    // Read the rest of the file
    while (std::getline(file, line))
    {
        // Create a string stream from the line
        std::stringstream ss(line);
        std::string value;
        std::vector<double> values;

        // Parse the line and extract values separated by commas
        while (std::getline(ss, value, ','))
        {
            try
            {
                values.push_back(std::stod(value));
            }
            catch (const std::invalid_argument& e)
            {
                std::cerr << "Invalid number in line: " << line << ", error: " << e.what() << std::endl;
                break;
            }
            catch (const std::out_of_range& e)
            {
                std::cerr << "Number out of range in line: " << line << ", error: " << e.what() << std::endl;
                break;
            }
        }

        if (values.size() >= 12)
        {
            std::array<double, 3> angles_radians = anglesToUnitVector(values[3], values[4], values[5]);
            KDTree::Point newpt(angles_radians[0], angles_radians[1], angles_radians[2], values[3], values[4], values[5], values[6], values[7], values[8], values[9], values[10], values[11]);
            points.emplace_back(newpt);
        }
        else
        {
            std::cerr << "Invalid line (expected 23 values): " << line << std::endl;
        }
    }

    file.close();
}

void KDTree::initialize(std::string _calibration_pt_file)
{
    std::vector<KDTree::Point> points;
    readYawPitchRollFromCSV(_calibration_pt_file, points);
    _root = buildKDTree(points.begin(), points.end(), 0);
}

/*void KDTree::get_points(std::vector<std::pair<double, KDTree::Point>>& nearestNeighbors, KDTree::Point& target)
{
    // Find 4 nearest neighbors
    int k = 4;
    std::priority_queue<std::pair<double, KDTree::Point>, std::vector<std::pair<double, KDTree::Point>>, KDTree::CompareDist> maxHeap;
    kNearestNeighbors(_root, target, k, 0, maxHeap);

    // Collect results from the max heap
    while (!maxHeap.empty())
    {
        nearestNeighbors.push_back(maxHeap.top());
        maxHeap.pop();
    }
};

double KDTree::get_total_dist(std::vector<std::pair<double, KDTree::Point>>& nearestNeighbors)
{
    // Output results (closest first)
    double total_dist = 0;
    for (auto it = nearestNeighbors.rbegin(); it != nearestNeighbors.rend(); ++it)
    {
        double dist = std::sqrt(it->first);
        total_dist += dist;
    }
    return total_dist;
};*/

std::array<double, 6> KDTree::get_ft_offset(double yaw, double pitch, double roll)
{
    // Target point
    std::array<double, 3> angles_radians = anglesToUnitVector(yaw, pitch, roll);
    KDTree::Point target(angles_radians[0], angles_radians[1], angles_radians[2]);

    // Find 4 nearest neighbors
    std::vector<std::pair<double, KDTree::Point>> nearestNeighbors;
    int k = 4;
    std::priority_queue<std::pair<double, KDTree::Point>, std::vector<std::pair<double, KDTree::Point>>, KDTree::CompareDist> maxHeap;
    kNearestNeighbors(_root, target, k, 0, maxHeap);

    // Collect results from the max heap
    while (!maxHeap.empty())
    {
        nearestNeighbors.push_back(maxHeap.top());
        maxHeap.pop();
    }

    // Output results (closest first)
    double total_dist = 0;
    for (auto it = nearestNeighbors.rbegin(); it != nearestNeighbors.rend(); ++it)
    {
        double dist = std::sqrt(it->first);
        total_dist += dist;
    }

    // Offset weighted avg for F (x,y,z) and T (x,y,z)
    std::array<double, 6> offsets = { 0 };

    for (auto it = nearestNeighbors.rbegin(); it != nearestNeighbors.rend(); ++it)
    {
        double dist = std::sqrt(it->first);
        const KDTree::Point& pt = it->second;

        offsets[0] += (dist / total_dist) * pt.fx;
        offsets[1] += (dist / total_dist) * pt.fy;
        offsets[2] += (dist / total_dist) * pt.fz;
        offsets[3] += (dist / total_dist) * pt.tx;
        offsets[4] += (dist / total_dist) * pt.ty;
        offsets[5] += (dist / total_dist) * pt.tz;
    }

    return offsets;
}

std::array<double, 3> KDTree::get_ypr_offsets(double yaw, double pitch, double roll, std::array<double, 6>& ft_vector)
{
    // Target point
    std::array<double, 3> angles_radians = anglesToUnitVector(yaw, pitch, roll);
    KDTree::Point target(yaw, pitch, roll, angles_radians[0], angles_radians[1], angles_radians[2], ft_vector[0], ft_vector[1], ft_vector[2], ft_vector[3], ft_vector[4], ft_vector[5]);

    // Find 4 nearest neighbors
    std::vector<std::pair<double, KDTree::Point>> nearestNeighbors;
    int k = 4;
    std::priority_queue<std::pair<double, KDTree::Point>, std::vector<std::pair<double, KDTree::Point>>, KDTree::CompareDist> maxHeap;
    kNearestNeighbors_ypr(_root, target, k, 0, maxHeap);

    // Collect results from the max heap
    while (!maxHeap.empty())
    {
        nearestNeighbors.push_back(maxHeap.top());
        maxHeap.pop();
    }

    // Output results (closest first)
    double total_dist = 0;
    for (auto it = nearestNeighbors.rbegin(); it != nearestNeighbors.rend(); ++it)
    {
        double dist = std::sqrt(it->first);
        total_dist += dist;
    }

    // Offset weighted avg for roll, pitch, yaw
    std::array<double, 3> offsets = { 0 };

    for (auto it = nearestNeighbors.rbegin(); it != nearestNeighbors.rend(); ++it)
    {
        double dist = std::sqrt(it->first);
        const KDTree::Point& pt = it->second;

        offsets[0] += (dist / total_dist) * pt.yaw_d;
        offsets[1] += (dist / total_dist) * pt.pitch_d;
        offsets[2] += (dist / total_dist) * pt.roll_d;
    }

    offsets[0] = 0;
    offsets[1] = 0;
    offsets[2] = 0;

    return offsets;
}