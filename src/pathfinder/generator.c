#include "okapi/pathfinder/include/pathfinder.h"

#include <stdlib.h>

int pathfinder_prepare(Waypoint *path, int path_length, void (*fit)(Waypoint,Waypoint,Spline*), int sample_count, double dt,
        double max_velocity, double max_acceleration, double max_jerk, TrajectoryCandidate *cand) {
    if (path_length < 2) return -1;
    
    cand->saptr = malloc((path_length - 1) * sizeof(Spline));
    cand->laptr = malloc((path_length - 1) * sizeof(double));
    double totalLength = 0;
    
    int i;
    for (i = 0; i < path_length-1; i++) {
        Spline s;
        fit(path[i], path[i+1], &s);
        double dist = pf_spline_distance(&s, sample_count);
        cand->saptr[i] = s;
        cand->laptr[i] = dist;
        totalLength += dist;
    }
    
    TrajectoryConfig config = {dt, max_velocity, max_acceleration, max_jerk, 0, path[0].angle,
        totalLength, 0, path[0].angle, sample_count};
    TrajectoryInfo info = pf_trajectory_prepare(config);
    int trajectory_length = info.length;
    
    cand->totalLength = totalLength;
    cand->length = trajectory_length;
    cand->path_length = path_length;
    cand->info = info;
    cand->config = config;
    
    return 0;
}

int pathfinder_generate(TrajectoryCandidate *c, Segment *segments) {
    int trajectory_length = c->length;
    int path_length = c->path_length;
    double totalLength = c->totalLength;
    
    Spline *splines = (c->saptr);
    double *splineLengths = (c->laptr);
    
    int trajectory_status = pf_trajectory_create(c->info, c->config, segments);
    if (trajectory_status < 0) return trajectory_status;
    
    int spline_i = 0;
    double spline_pos_initial = 0, splines_complete = 0;
    
    int i;
    for (i = 0; i < trajectory_length; ++i) {
        double pos = segments[i].position;

        int found = 0;
        while (!found) {
            double pos_relative = pos - spline_pos_initial;
            if (pos_relative <= splineLengths[spline_i]) {
                Spline si = splines[spline_i];
                double percentage = pf_spline_progress_for_distance(si, pos_relative, c->config.sample_count);
                Coord coords = pf_spline_coords(si, percentage);
                segments[i].heading = pf_spline_angle(si, percentage);
                segments[i].x = coords.x;
                segments[i].y = coords.y;
                found = 1;
            } else if (spline_i < path_length - 2) {
                splines_complete += splineLengths[spline_i];
                spline_pos_initial = splines_complete;
                spline_i += 1;
            } else {
                Spline si = splines[path_length - 2];
                segments[i].heading = pf_spline_angle(si, 1.0);
                Coord coords = pf_spline_coords(si, 1.0);
                segments[i].x = coords.x;
                segments[i].y = coords.y;
                found = 1;
            }
        }
    }
    
    free(c->saptr);
    free(c->laptr);
    
    return trajectory_length;
}