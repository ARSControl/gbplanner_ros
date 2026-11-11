// graph_merging/connected_components.h
#pragma once

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace merge {

struct ComponentsInfo {
    int num_vertices = 0;
    int num_components = 0;
    int largest_component = 0;   // size (in vertices) of the biggest component
};

namespace detail {

// Disjoint-set (union-find) with path compression and union by size.
class UnionFind {
public:
    explicit UnionFind(int n) : parent_(n), size_(n, 1) {
        for (int i = 0; i < n; ++i) parent_[i] = i;
    }
    int find(int x) {
        // iterative, path halving (no recursion -> safe on big graphs)
        while (parent_[x] != x) {
            parent_[x] = parent_[parent_[x]];
            x = parent_[x];
        }
        return x;
    }
    void unite(int a, int b) {
        a = find(a);
        b = find(b);
        if (a == b) return;
        if (size_[a] < size_[b]) std::swap(a, b);
        parent_[b] = a;
        size_[a] += size_[b];
    }
    int componentSize(int root) const { return size_[root]; }
private:
    std::vector<int> parent_;
    std::vector<int> size_;
};

}  // namespace detail

// Counts the connected components of a graph. Edges are treated as undirected.
// GraphT must expose: vertices_map_ (id -> Vertex*) and
// getAdjacentVertices(Vertex*, std::vector<int>&).
template <typename GraphT>
ComponentsInfo countConnectedComponents(const std::shared_ptr<GraphT>& graph) {
    ComponentsInfo info;
    if (!graph) return info;

    const int n = static_cast<int>(graph->vertices_map_.size());
    info.num_vertices = n;
    if (n == 0) return info;

    // Vertex ids are not necessarily contiguous -> map them to dense indices 0..n-1
    std::unordered_map<int, int> id_to_idx;
    id_to_idx.reserve(n);
    int idx = 0;
    for (const auto& kv : graph->vertices_map_) id_to_idx[kv.first] = idx++;

    detail::UnionFind uf(n);
    std::vector<int> neighbours;
    for (const auto& kv : graph->vertices_map_) {
        neighbours.clear();
        graph->getAdjacentVertices(kv.second, neighbours);
        const int a = id_to_idx[kv.first];
        for (int nid : neighbours) {
            auto it = id_to_idx.find(nid);
            if (it == id_to_idx.end()) continue;   // dangling edge, ignore
            uf.unite(a, it->second);
        }
    }

    for (int i = 0; i < n; ++i) {
        if (uf.find(i) == i) {                     // i is a root -> one component
            ++info.num_components;
            info.largest_component =
                std::max(info.largest_component, uf.componentSize(i));
        }
    }
    return info;
}

// Appends one CSV line to `filename` (header written if the file is new/empty).
inline void logMergeComponents(const std::string& filename,
                               const std::string& merge_type,   // "self_merge" / "two_robot_merge"
                               int robot_id,
                               const ComponentsInfo& before,
                               const ComponentsInfo& after) {
    static std::mutex mtx;                          // callbacks may run in different threads
    std::lock_guard<std::mutex> lock(mtx);

    std::ofstream file(filename, std::ios::app);
    if (!file.is_open()) {
        std::cerr << "Failed to open components log file: " << filename << std::endl;
        return;
    }
    file.seekp(0, std::ios::end);
    if (file.tellp() == 0) {
        file << "timestamp,robot_id,merge_type,"
                "vertices_before,components_before,largest_before,"
                "vertices_after,components_after,largest_after,"
                "components_delta,connectivity_worsened\n";
    }

    const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    const int delta = after.num_components - before.num_components;
    file << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S") << ","
         << robot_id << "," << merge_type << ","
         << before.num_vertices << "," << before.num_components << "," << before.largest_component << ","
         << after.num_vertices  << "," << after.num_components  << "," << after.largest_component  << ","
         << delta << "," << (delta > 0 ? "YES" : "NO") << "\n";
}

}  // namespace merge