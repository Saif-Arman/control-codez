#include "KDTree.h"

#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <array>
#include "Global.h"

//#define M_PI 3.14159265358979323846264338

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
            return a.yaw < b.yaw;
        else if (axis == 1)
            return a.pitch < b.pitch;
        else
            return a.roll < b.roll;
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
    return (a.yaw - b.yaw) * (a.yaw - b.yaw) +
        (a.pitch - b.pitch) * (a.pitch - b.pitch) +
        (a.roll - b.roll) * (a.roll - b.roll);
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
        maxHeap.emplace(dist, root->point);
    }
    // If current point is closer than the farthest point in heap, replace it
    else if (dist < maxHeap.top().first)
    {
        maxHeap.pop();
        maxHeap.emplace(dist, root->point);
    }

    // Determine which side to search first
    int axis = depth % 3;
    double diff;
    if (axis == 0)
        diff = target.yaw - root->point.yaw;
    else if (axis == 1)
        diff = target.pitch - root->point.pitch;
    else
        diff = target.roll - root->point.roll;

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
    // Convert angles from degrees to radians
    double yawRad = yaw * M_PI / 180.0;
    double pitchRad = pitch * M_PI / 180.0;
    double rollRad = roll * M_PI / 180.0;

    // Compute unit vector components
    double x = std::cos(yawRad) * std::cos(pitchRad);
    double y = std::sin(yawRad) * std::cos(pitchRad);
    double z = std::sin(pitchRad);

    std::array<double, 3> result = { x, y, z };
    return result;

    //return { x, y, z };
}

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
                std::cerr << "Invalid number in line: " << line << std::endl;
                break;
            }
            catch (const std::out_of_range& e)
            {
                std::cerr << "Number out of range in line: " << line << std::endl;
                break;
            }
        }

        // Ensure that we have exactly three values: yaw, pitch, roll
        if (values.size() == 23)
        {
            std::array<double, 3> angles = anglesToUnitVector(values[3], values[4], values[5]);
            /*points.emplace_back(angles[0], angles[1], angles[2], values[3], values[4], values[5], values[6], values[7], values[8], values[9], values[10], values[11]);*/
            KDTree::Point newpt(angles[0], angles[1], angles[2], values[3], values[4], values[5], values[6], values[7], values[8], values[9], values[10], values[11]);
            points.emplace_back(newpt);
        }
        else
        {
            std::cerr << "Invalid line (expected 23 values): " << line << std::endl;
        }
    }

    file.close();
}

//// Example usage
//int main()
//{
//    std::string filename = "C:\\Users\\nick_\\Documents\\MSCpE\\Behal_Research\\KD_tree\\calibration_cloud_240pts.csv";
//
//    std::vector<Point> points;
//    readYawPitchRollFromCSV(filename, points);
//
//    // Build KD-tree
//    KDNode* root = buildKDTree(points.begin(), points.end(), 0);
//
//    // Target point
//    std::array<double, 3> angles = anglesToUnitVector(172, 38, 173);
//    Point target(angles[0], angles[1], angles[2]);
//
//    // Find 4 nearest neighbors
//    int k = 4;
//    std::priority_queue<std::pair<double, Point>,
//        std::vector<std::pair<double, Point>>,
//        CompareDist>
//        maxHeap;
//    kNearestNeighbors(root, target, k, 0, maxHeap);
//
//    // Collect results from the max heap
//    std::vector<std::pair<double, Point>> nearestNeighbors;
//    while (!maxHeap.empty())
//    {
//        nearestNeighbors.push_back(maxHeap.top());
//        maxHeap.pop();
//    }
//
//    // Output results (closest first)
//    std::cout << "The " << k << " nearest neighbors are:\n";
//    for (auto it = nearestNeighbors.rbegin(); it != nearestNeighbors.rend(); ++it)
//    {
//        const Point& pt = it->second;
//        std::cout << "Yaw: " << pt.yaw_vec << ", Pitch: " << pt.pitch_vec
//            << ", Roll: " << pt.roll_vec << ", Distance(linear): " << std::sqrt(it->first) << "\n";
//    }
//
//    // Cleanup
//    deleteKDTree(root);
//
//    return 0;
//}
