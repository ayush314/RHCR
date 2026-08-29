#include "WorkstationGraph.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

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

void set_cardinal_weights(WorkstationGrid& G, const vector<bool>& blocked = {})
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
            bool is_blocked = !blocked.empty() && blocked[loc];
            if (is_blocked)
            {
                G.types[loc] = "Obstacle";
                continue;
            }
            G.weights[loc][4] = 1;
            if (x + 1 < G.cols && (blocked.empty() || !blocked[loc + 1]))
                G.weights[loc][0] = 1;
            if (y - 1 >= 0 && (blocked.empty() || !blocked[loc - G.cols]))
                G.weights[loc][1] = 1;
            if (x - 1 >= 0 && (blocked.empty() || !blocked[loc - 1]))
                G.weights[loc][2] = 1;
            if (y + 1 < G.rows && (blocked.empty() || !blocked[loc + G.cols]))
                G.weights[loc][3] = 1;
        }
    }
}

string resolve_relative_path(const string& benchmark_path, const string& referenced_path)
{
    if (referenced_path.empty() || referenced_path.front() == '/')
        return referenced_path;
    size_t slash = benchmark_path.find_last_of("/\\");
    return slash == string::npos
        ? referenced_path
        : benchmark_path.substr(0, slash + 1) + referenced_path;
}

bool load_movingai_grid(WorkstationGrid& G, const string& path)
{
    std::ifstream input(path.c_str());
    if (!input.is_open())
    {
        std::cout << "MovingAI map " << path << " does not exist." << std::endl;
        return false;
    }

    string label;
    string type;
    if (!(input >> label >> type) || label != "type" ||
        !(input >> label >> G.rows) || label != "height" ||
        !(input >> label >> G.cols) || label != "width" ||
        !(input >> label) || label != "map" || G.rows <= 0 || G.cols <= 0)
    {
        std::cout << "Invalid MovingAI map header in " << path << std::endl;
        return false;
    }
    string line;
    std::getline(input, line);

    vector<bool> blocked(G.rows * G.cols, false);
    for (int y = 0; y < G.rows; y++)
    {
        if (!std::getline(input, line) || (int)line.size() != G.cols)
        {
            std::cout << "Invalid MovingAI map row " << y << " in " << path << std::endl;
            return false;
        }
        for (int x = 0; x < G.cols; x++)
        {
            char cell = line[x];
            blocked[G.to_id(x, y)] = cell == '@' || cell == 'O' || cell == 'T' || cell == 'W';
        }
    }
    set_cardinal_weights(G, blocked);
    return true;
}
} // namespace

WorkstationGrid::~WorkstationGrid()
{
    if (compact_mapping != nullptr)
        munmap(compact_mapping, compact_mapping_size);
}

bool WorkstationGrid::load_map(string fname)
{
    zone_index_complete = false;
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
    auto movingai_map = root.get_optional<string>("movingai_map");
    if (movingai_map)
    {
        string source_path = resolve_relative_path(fname, *movingai_map);
        movingai_map_path = source_path;
        if (!load_movingai_grid(*this, source_path))
            return false;
        int expected_rows = root.get<int>("rows", rows);
        int expected_cols = root.get<int>("cols", cols);
        if (rows != expected_rows || cols != expected_cols)
        {
            std::cout << "Benchmark dimensions do not match " << source_path << std::endl;
            return false;
        }
    }
    else
    {
        rows = root.get<int>("rows");
        cols = root.get<int>("cols");
        set_cardinal_weights(*this);
    }
    zone_cell_to_station_dense.assign(size(), -1);

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
        zone_cell_to_station_dense[station.workstation] = station.station_id;
        types[station.workstation] = "Workstation";

        for (const auto& child : station_node.get_child("standby_cells"))
        {
            int loc = as_loc(*this, child.second);
            station.standby_cells.push_back(loc);
            station.zone_cells.insert(loc);
            zone_cell_to_station[loc] = station.station_id;
            zone_cell_to_station_dense[loc] = station.station_id;
            types[loc] = "Standby";
        }
        for (const auto& child : station_node.get_child("buffer_cells"))
        {
            int loc = as_loc(*this, child.second);
            station.buffer_cells.push_back(loc);
            station.zone_cells.insert(loc);
            zone_cell_to_station[loc] = station.station_id;
            zone_cell_to_station_dense[loc] = station.station_id;
            types[loc] = "Buffer";
        }
        for (const auto& child : station_node.get_child("approach_cells"))
        {
            int loc = as_loc(*this, child.second);
            station.approach_cells.push_back(loc);
            station.zone_cells.insert(loc);
            zone_cell_to_station[loc] = station.station_id;
            zone_cell_to_station_dense[loc] = station.station_id;
            types[loc] = "Approach";
        }
        for (const auto& child : station_node.get_child("exit_cells"))
        {
            int loc = as_loc(*this, child.second);
            station.exit_cells.push_back(loc);
            station.zone_cells.insert(loc);
            exit_to_station[loc] = station.station_id;
            zone_cell_to_station[loc] = station.station_id;
            zone_cell_to_station_dense[loc] = station.station_id;
            types[loc] = "Exit";
        }

        stations.push_back(station);
    }
    zone_index_complete = true;

    unordered_set<int> reserved = endpoint_set;
    for (const auto& station : stations)
    {
        reserved.insert(station.zone_cells.begin(), station.zone_cells.end());
    }
    for (int loc = 0; loc < size(); loc++)
    {
        if (weights[loc][4] < WEIGHT_MAX && reserved.find(loc) == reserved.end())
            free_start_cells.push_back(loc);
    }

    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Map size: " << rows << "x" << cols << " with "
              << pickup_endpoints.size() << " endpoints and "
              << stations.size() << " workstations." << std::endl;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
    return true;
}

unordered_set<int> WorkstationGrid::workstation_goal_roots() const
{
    unordered_set<int> roots = endpoint_set;
    for (const auto& station : stations)
    {
        roots.insert(station.workstation);
        roots.insert(station.exit_cells.begin(), station.exit_cells.end());
    }
    return roots;
}

void WorkstationGrid::preprocessing(bool consider_rotation)
{
    std::cout << "*** PreProcessing workstation map ***" << std::endl;
    clock_t t = std::clock();
    this->consider_rotation = consider_rotation;
    string fname = map_name + (consider_rotation ? "_rotation_heuristics_table.txt" : "_heuristics_table.txt");
    unordered_set<int> goal_roots = workstation_goal_roots();
    bool succ = false;
    std::ifstream in(fname.c_str());
    if (in.is_open())
    {
        succ = load_heuristics_table(in);
        in.close();
        if (succ)
        {
            succ = std::all_of(goal_roots.begin(), goal_roots.end(), [&](int root) {
                return heuristics.find(root) != heuristics.end();
            });
            if (!succ)
                heuristics.clear();
        }
    }
    if (!succ)
    {
        for (int root : goal_roots)
        {
            heuristics[root] = compute_heuristics(root);
        }
        save_heuristics_table(fname);
    }
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
}

string WorkstationGrid::compact_heuristic_path() const
{
    string stem = movingai_map_path;
    size_t dot = stem.rfind('.');
    if (dot != string::npos)
        stem.erase(dot);
    return stem + "_p100_compact_heuristics.bin";
}

string WorkstationGrid::full_text_heuristic_path() const
{
    string stem = movingai_map_path;
    size_t dot = stem.rfind('.');
    if (dot != string::npos)
        stem.erase(dot);
    return stem + "_p100_heuristics_table.txt";
}

void WorkstationGrid::build_compact_heuristics(const string& text_path,
                                               const string& binary_path) const
{
    std::ifstream input(text_path.c_str());
    if (!input.is_open())
        throw std::runtime_error("Missing exact heuristic source: " + text_path);

    string line;
    std::getline(input, line);
    std::getline(input, line);
    size_t comma = line.find(',');
    if (comma == string::npos)
        throw std::runtime_error("Invalid heuristic header: " + text_path);
    uint32_t root_count = (uint32_t)std::stoul(line.substr(0, comma));
    uint32_t map_size = (uint32_t)std::stoul(line.substr(comma + 1));
    if (map_size != (uint32_t)size())
        throw std::runtime_error("Heuristic map size mismatch: " + text_path);

    const string temporary = binary_path + ".tmp." + std::to_string((long long)getpid());
    std::ofstream output(temporary.c_str(), std::ios::binary | std::ios::trunc);
    if (!output.is_open())
        throw std::runtime_error("Cannot create compact heuristic table: " + temporary);

    const char magic[8] = {'W', 'S', 'H', '1', '6', '\0', '\0', '\1'};
    output.write(magic, sizeof(magic));
    output.write(reinterpret_cast<const char*>(&map_size), sizeof(map_size));
    output.write(reinterpret_cast<const char*>(&root_count), sizeof(root_count));
    vector<uint16_t> row(map_size);
    for (uint32_t index = 0; index < root_count; index++)
    {
        if (!std::getline(input, line))
            throw std::runtime_error("Truncated heuristic roots: " + text_path);
        int32_t root = (int32_t)std::stoi(line);
        if (!std::getline(input, line))
            throw std::runtime_error("Truncated heuristic row: " + text_path);

        const char* cursor = line.c_str();
        const char* end = cursor + line.size();
        for (uint32_t cell = 0; cell < map_size; cell++)
        {
            uint32_t value = 0;
            bool integer_value = cursor < end;
            while (cursor < end && *cursor != ',')
            {
                if (*cursor >= '0' && *cursor <= '9' && integer_value)
                {
                    value = std::min<uint32_t>(65535, value * 10 + (*cursor - '0'));
                }
                else
                {
                    integer_value = false;
                }
                cursor++;
            }
            if (cursor < end && *cursor == ',')
                cursor++;
            row[cell] = integer_value && value < 65535 ? (uint16_t)value : UINT16_MAX;
        }
        output.write(reinterpret_cast<const char*>(&root), sizeof(root));
        output.write(reinterpret_cast<const char*>(row.data()), row.size() * sizeof(uint16_t));
        if (!output)
            throw std::runtime_error("Failed writing compact heuristic table: " + temporary);
    }
    output.close();
    if (std::rename(temporary.c_str(), binary_path.c_str()) != 0)
    {
        std::remove(temporary.c_str());
        throw std::runtime_error("Cannot publish compact heuristic table: " +
                                 string(std::strerror(errno)));
    }
}

bool WorkstationGrid::load_compact_heuristics(const string& path)
{
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0)
        return false;
    struct stat info;
    if (fstat(fd, &info) != 0 || info.st_size < 16)
    {
        close(fd);
        return false;
    }
    void* mapping = mmap(nullptr, info.st_size, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (mapping == MAP_FAILED)
        return false;

    const char expected[8] = {'W', 'S', 'H', '1', '6', '\0', '\0', '\1'};
    const char* bytes = static_cast<const char*>(mapping);
    uint32_t map_size = 0;
    uint32_t root_count = 0;
    std::memcpy(&map_size, bytes + 8, sizeof(map_size));
    std::memcpy(&root_count, bytes + 12, sizeof(root_count));
    const size_t record_size = sizeof(int32_t) + (size_t)map_size * sizeof(uint16_t);
    const size_t expected_size = 16 + (size_t)root_count * record_size;
    if (std::memcmp(bytes, expected, sizeof(expected)) != 0 ||
        map_size != (uint32_t)size() || expected_size != (size_t)info.st_size)
    {
        munmap(mapping, info.st_size);
        return false;
    }

    compact_mapping = mapping;
    compact_mapping_size = info.st_size;
    compact_heuristics.clear();
    compact_heuristics.reserve(root_count);
    const char* record = bytes + 16;
    for (uint32_t index = 0; index < root_count; index++, record += record_size)
    {
        int32_t root = -1;
        std::memcpy(&root, record, sizeof(root));
        compact_heuristics[root] = reinterpret_cast<const uint16_t*>(record + sizeof(root));
    }
    return true;
}

void WorkstationGrid::preprocessing_compact(bool consider_rotation)
{
    if (movingai_map_path.empty() || consider_rotation)
    {
        preprocessing(consider_rotation);
        return;
    }
    std::cout << "*** Memory-mapping exact workstation heuristics ***" << std::endl;
    clock_t t = std::clock();
    this->consider_rotation = consider_rotation;
    const string binary_path = compact_heuristic_path();
    if (!load_compact_heuristics(binary_path))
    {
        std::cout << "Building compact exact table from " << full_text_heuristic_path() << std::endl;
        build_compact_heuristics(full_text_heuristic_path(), binary_path);
        if (!load_compact_heuristics(binary_path))
            throw std::runtime_error("Failed to load compact heuristic table: " + binary_path);
    }
    for (int root : workstation_goal_roots())
    {
        if (compact_heuristics.find(root) == compact_heuristics.end())
            throw std::runtime_error("Compact heuristic table is missing required goal root");
    }
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Done! (" << runtime << " s, " << compact_heuristics.size()
              << " exact roots)" << std::endl;
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
    if (zone_index_complete && loc >= 0 &&
        loc < static_cast<int>(zone_cell_to_station_dense.size()))
    {
        return zone_cell_to_station_dense[loc];
    }
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
    auto compact = compact_heuristics.find(to);
    if (compact != compact_heuristics.end() && from >= 0 && from < size())
    {
        uint16_t distance = compact->second[from];
        return distance == UINT16_MAX ? INT_MAX / 4 : (int)distance;
    }
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
