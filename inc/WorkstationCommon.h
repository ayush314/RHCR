#pragma once

enum class WorkstationAgentPhase
{
    NONE = 0,
    TO_PICKUP = 1,
    TO_STATION = 2,
    TO_EXIT = 3,
    SERVICE = 4,
};

struct WorkstationAgentContext
{
    int station_id = -1;
    int boundary_entry_t = -1;
    int task_issue_t = -1;
    WorkstationAgentPhase phase = WorkstationAgentPhase::NONE;
};
