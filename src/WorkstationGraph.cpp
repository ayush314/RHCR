#include "WorkstationGraph.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <climits>
#include <sstream>

using boost::property_tree::ptree;

namespace
{
int as_loc(const WorkstationGrid& G, const ptree& node)
{
    auto it = node.begin();
    int x = std::stoi(it->second.data());
    ++it;
    int y = std::stoi(it->second.data());
    return G.to_id(x, y);
}

void set_cardinal_weights(WorkstationGrid& G)
{
    G.move[0] = 1;
    G.move[1] = -G.cols;
    G.move[2] = -1;
    G.move[3] = G.cols;
    G.weights.assign(G.size(), vector<double>(5, WEIGHT_MAX));
    G.types.assign(G.size(), "Empty");
    for (int y = 0; y < G.rows; y++)
    {
        for (int x = 0; x < G.cols; x++)
        {
            int loc = G.to_id(x, y);
            G.weights[loc][4] = 1;
            if (x + 1 < G.cols)
                G.weights[loc][0] = 1;
            if (y - 1 >= 0)
                G.weights[loc][1] = 1;
            if (x - 1 >= 0)
                G.weights[loc][2] = 1;
            if (y + 1 < G.rows)
                G.weights[loc][3] = 1;
        }
    }
}
} // namespace

bool WorkstationGrid::load_map(string fname)
{
    std::ifstream stream(fname.c_str());
    if (!stream.is_open())
    {
        std::cout << "Benchmark file " << fname << " does not exist." << std::endl;
        return false;
    }

    std::cout << "*** Loading workstation benchmark ***" << std::endl;
    clock_t t = std::clock();
    std::size_t pos = fname.rfind('.');
    map_name = fname.substr(0, pos);

    ptree root;
    read_json(stream, root);
    rows = root.get<int>("rows");
    cols = root.get<int>("cols");
    set_cardinal_weights(*this);

    for (const auto& child : root.get_child("pickup_endpoints"))
    {
        int loc = as_loc(*this, child.second);
        pickup_endpoints.push_back(loc);
        endpoint_set.insert(loc);
        types[loc] = "Endpoint";
    }

    for (const auto& station_child : root.get_child("stations"))
    {
        const ptree& station_node = station_child.second;
        WorkstationStation station;
        station.station_id = station_node.get<int>("station_id");
        station.workstation = as_loc(*this, station_node.get_child("workstation_cell"));
        station.zone_cells.insert(station.workstation);
        workstation_to_station[station.workstation] = station.station_id;
        zone_cell_to_station[station.workstation] = station.station_id;
        types[station.workstation] = "Workstation";

        for (const auto& child : station_node.get_child("standby_cells"))
        {
            int loc = as_loc(*this, child.second);
            station.standby_cells.push_back(loc);
            station.zone_cells.insert(loc);
            zone_cell_to_station[loc] = station.station_id;
            types[loc] = "Standby";
        }
        for (const auto& child : station_node.get_child("buffer_cells"))
        {
            int loc = as_loc(*this, child.second);
            station.buffer_cells.push_back(loc);
            station.zone_cells.insert(loc);
            zone_cell_to_station[loc] = station.station_id;
            types[loc] = "Buffer";
        }
        for (const auto& child : station_node.get_child("approach_cells"))
        {
            int loc = as_loc(*this, child.second);
            station.approach_cells.push_back(loc);
            station.zone_cells.insert(loc);
            zone_cell_to_station[loc] = station.station_id;
            types[loc] = "Approach";
        }
        for (const auto& child : station_node.get_child("exit_cells"))
        {
            int loc = as_loc(*this, child.second);
            station.exit_cells.push_back(loc);
            station.zone_cells.insert(loc);
            exit_to_station[loc] = station.station_id;
            zone_cell_to_station[loc] = station.station_id;
            types[loc] = "Exit";
        }

        stations.push_back(station);
    }

    unordered_set<int> reserved = endpoint_set;
    for (const auto& station : stations)
    {
        reserved.insert(station.zone_cells.begin(), station.zone_cells.end());
    }
    for (int loc = 0; loc < size(); loc++)
    {
        if (reserved.find(loc) == reserved.end())
            free_start_cells.push_back(loc);
    }

    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Map size: " << rows << "x" << cols << " with "
              << pickup_endpoints.size() << " endpoints and "
              << stations.size() << " workstations." << std::endl;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
    return true;
}

void WorkstationGrid::preprocessing(bool consider_rotation)
{
    std::cout << "*** PreProcessing workstation map ***" << std::endl;
    clock_t t = std::clock();
    this->consider_rotation = consider_rotation;
    string fname = map_name + (consider_rotation ? "_rotation_heuristics_table.txt" : "_heuristics_table.txt");
    bool succ = false;
    std::ifstream in(fname.c_str());
    if (in.is_open())
    {
        succ = load_heuristics_table(in);
        in.close();
    }
    if (!succ)
    {
        unordered_set<int> goal_roots = endpoint_set;
        for (const auto& station : stations)
        {
            goal_roots.insert(station.workstation);
            goal_roots.insert(station.exit_cells.begin(), station.exit_cells.end());
        }
        for (int root : goal_roots)
        {
            heuristics[root] = compute_heuristics(root);
        }
        save_heuristics_table(fname);
    }
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
}

int WorkstationGrid::station_for_workstation(int loc) const
{
    auto it = workstation_to_station.find(loc);
    return it == workstation_to_station.end() ? -1 : it->second;
}

int WorkstationGrid::station_for_exit(int loc) const
{
    auto it = exit_to_station.find(loc);
    return it == exit_to_station.end() ? -1 : it->second;
}

int WorkstationGrid::station_for_zone_cell(int loc) const
{
    auto it = zone_cell_to_station.find(loc);
    return it == zone_cell_to_station.end() ? -1 : it->second;
}

bool WorkstationGrid::conflict_in_station_microzone(int station_id, int v1, int v2) const
{
    auto in_station = [&](int loc) {
        return loc >= 0 && station_for_zone_cell(loc) == station_id;
    };
    return in_station(v1) || in_station(v2);
}

int WorkstationGrid::choose_exit_for_endpoint(int station_id, int endpoint_loc) const
{
    const auto& exits = stations[station_id].exit_cells;
    int best_exit = exits.front();
    int best_dist = INT_MAX / 2;
    for (int exit_loc : exits)
    {
        int dist = distance_between(exit_loc, endpoint_loc);
        if (dist < best_dist || (dist == best_dist && exit_loc < best_exit))
        {
            best_dist = dist;
            best_exit = exit_loc;
        }
    }
    return best_exit;
}

int WorkstationGrid::distance_to_workstation(int station_id, int loc) const
{
    return distance_between(loc, stations[station_id].workstation);
}

int WorkstationGrid::distance_between(int from, int to) const
{
    auto it = heuristics.find(to);
    if (it == heuristics.end() || from < 0 || from >= (int)it->second.size())
        return get_Manhattan_distance(from, to);
    return (int)it->second[from];
}

void WorkstationGrid::append_workstation_dwell_goals(vector<pair<int, int> >& goals, int station_id, int dwell_steps) const
{
    int workstation = stations[station_id].workstation;
    for (int i = 0; i <= dwell_steps; i++)
    {
        goals.emplace_back(workstation, i);
    }
}
