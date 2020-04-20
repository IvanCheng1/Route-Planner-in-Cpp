#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}


// Implement the CalculateHValue method.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return (*node).distance(*end_node);

}


// AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    (*current_node).FindNeighbors();

    for (auto neighbor : (*current_node).neighbors) {
        (*neighbor).parent = current_node;
        (*neighbor).g_value = (*current_node).g_value + (*current_node).distance(*neighbor);
        (*neighbor).h_value = CalculateHValue(neighbor);

        open_list.push_back(neighbor);
        (*neighbor).visited = true;
    }
}


// NextNode method to sort the open list and return the next node.

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](const auto & one, const auto & two) {
        return ((*one).h_value + (*one).g_value > (*two).h_value + (*two).g_value);
    });

    RouteModel::Node *next_pointer = open_list.back();
    open_list.pop_back();
    return next_pointer;
}


// ConstructFinalPath method to return the final path found from your A* search.
// Path is back to front as it iterates from the end point to the start point

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector and reverse_path vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    std::vector<RouteModel::Node> reverse_path;

    while (current_node != start_node) {
        reverse_path.push_back((*current_node));
        RouteModel::Node parent = *(*current_node).parent;
        distance += (*current_node).distance(parent);
        current_node = (*current_node).parent;
    }

    reverse_path.push_back((*current_node));

    for (int i=reverse_path.size()-1; i >= 0; i--) {
        path_found.push_back(reverse_path[i]);
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// A* Search algorithm

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = start_node;
    (*start_node).visited = true;
    open_list.push_back(current_node);

    while (current_node != end_node) {
        AddNeighbors(current_node);
        current_node = NextNode();
    }

    m_Model.path = ConstructFinalPath(end_node);
}
