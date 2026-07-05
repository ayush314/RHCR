#pragma once

#include "BasicGraph.h"
#include "WorkstationCommon.h"

struct WorkstationStation
{
    int station_id = -1;
    int workstation = -1;
    vector<int> standby_cells;
    vector<int> buffer_cells;
    vector<int> approach_cells;
    vector<int> exit_cells;
    unordered_set<int> zone_cells;
};

class WorkstationGrid : public BasicGraph
{
public:
    vector<int> pickup_endpoints;
    vector<WorkstationStation> stations;
    vector<int> free_start_cells;

    bool load_map(string fname) override;
    void preprocessing(bool consider_rotation);

    int to_id(int x, int y) const { return y * cols + x; }
    pair<int, int> to_xy(int loc) const { return make_pair(loc % cols, loc / cols); }

    int station_for_workstation(int loc) const;
    int station_for_exit(int loc) const;
    int station_for_zone_cell(int loc) const;
    bool conflict_in_station_microzone(int station_id, int v1, int v2) const;

    int choose_exit_for_endpoint(int station_id, int endpoint_loc) const;
    int distance_to_workstation(int station_id, int loc) const;
    int distance_between(int from, int to) const;
    void append_workstation_dwell_goals(vector<pair<int, int> >& goals, int station_id, int dwell_steps) const;

private:
    unordered_set<int> endpoint_set;
    unordered_map<int, int> workstation_to_station;
    unordered_map<int, int> exit_to_station;
    unordered_map<int, int> zone_cell_to_station;
};
