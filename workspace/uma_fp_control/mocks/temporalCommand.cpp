/**
 * @file temporalMock.cpp
 * @brief Publishes a timed sequence of desired positions AND fulcrum perturbations.
 *
 * Two types of events run on the same timeline:
 *
 *   GoalEvent:       publishes a geometry_msgs::Point to /alice/coordinator/goal_position
 *   PerturbEvent:    publishes a Float64MultiArray [dx,dy,dz] to /fulcrum_perturbation
 *                    abdomenForceMock picks this up and shifts the virtual fulcrum.
 *
 * Timeline example (with default values):
 *
 *   t=0s   — node starts, recording begins
 *   t=5s   — punto[0] sent, perturbation OFF (0,0,0)
 *   t=15s  — punto[1] sent
 *   t=17s  — PERTURBATION ON: fulcrum shifts +5mm in X
 *   t=22s  — punto[2] sent
 *   t=25s  — PERTURBATION OFF: fulcrum returns to nominal
 *   t=32s  — punto[3] sent
 *   ...
 *
 * To design your experiment: edit the GoalEvent list and PerturbEvent list below.
 * All times are in seconds from node start.
 */

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64MultiArray.h"
#include <vector>

// ============================================================================
// Event types
// ============================================================================

struct GoalEvent {
    double               time;  // seconds from start
    geometry_msgs::Point p;
    bool                 sent = false;
};

struct PerturbEvent {
    double time;   // seconds from start
    double dx;     // fulcrum displacement X (m)
    double dy;     // fulcrum displacement Y (m)
    double dz;     // fulcrum displacement Z (m)
    bool   sent = false;
};


// ============================================================================
// Helper to build a Point
// ============================================================================
geometry_msgs::Point makePoint(double x, double y, double z){
    geometry_msgs::Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
}


// ============================================================================
// Main
// ============================================================================
int main(int argc, char **argv){
    ros::init(argc, argv, "temporalCommand");
    ros::NodeHandle nh;

    ros::Publisher pub_goal = nh.advertise<geometry_msgs::Point>(
        "/auto/coordinator/goal_position", 1000);

    ros::Publisher pub_goal_abdomen = nh.advertise<geometry_msgs::Point>(
        "/darel/coordinator/goal_position", 1000);

    ros::Publisher pub_perturb = nh.advertise<std_msgs::Float64MultiArray>(
        "/fulcrum_perturbation", 10);

    ros::Rate rate(125);

    // ========================================================================
    // GOAL EVENTS — desired instrument positions
    // ========================================================================
    // Edit these to match your experiment trajectory.
    const double start_delay = 5.0;   // seconds before first goal
    const double intervalo   = 10.0;  // seconds between goals

    /*std::vector<geometry_msgs::Point> puntos(6);
    puntos[0] = makePoint(0.16,  -0.276, -0.034); //74
    puntos[1] = makePoint(0.16,  -0.294, -0.014);
    puntos[2] = makePoint(0.16,  -0.317, -0.034);
    puntos[3] = makePoint(0.128, -0.273, -0.034); //68
    puntos[4] = makePoint(0.128, -0.294, -0.015);
    puntos[5] = makePoint(0.128, -0.317, -0.034);*/

    std::vector<geometry_msgs::Point> puntos(6);
    puntos[0] = makePoint(-0.245, 0.251, 0.019);
    puntos[1] = makePoint(-0.245, 0.251, 0.019);//-0.0525
    puntos[2] = makePoint(-0.242, 0.25, -0.003); 
    puntos[3] = makePoint(-0.245, 0.251, 0.019);
    puntos[4] = makePoint(-0.245, 0.251, 0.019); 
    puntos[5] = makePoint(-0.245, 0.251, 0.019);
    std::vector<geometry_msgs::Point> puntos_abdomen(6);
    puntos_abdomen[0] = makePoint(0.28, 0.253, -0.120); //186
    puntos_abdomen[1] = makePoint(0.278, 0.25, -0.142);
    puntos_abdomen[2] = makePoint(0.278, 0.25, -0.142); 
    puntos_abdomen[3] = makePoint(0.278, 0.25, -0.142);
    puntos_abdomen[4] = makePoint(0.28, 0.253, -0.120); 
    puntos_abdomen[5] = makePoint(0.28, 0.253, -0.120);


    std::vector<GoalEvent> goal_events;
    for (size_t i = 0; i < puntos.size(); i++){
        GoalEvent ev;
        ev.time = start_delay + i * intervalo;
        ev.p    = puntos[i];
        goal_events.push_back(ev);
    }

    std::vector<GoalEvent> goal_events_abdomen;
    for (size_t i = 0; i < puntos_abdomen.size(); i++){
        GoalEvent ev_abdomen;
        ev_abdomen.time = start_delay + i * intervalo;
        ev_abdomen.p    = puntos_abdomen[i];
        goal_events_abdomen.push_back(ev_abdomen);
    }

    // ========================================================================
    // PERTURBATION EVENTS — fulcrum displacement sequence
    // ========================================================================
    // Each entry sets the fulcrum offset from that moment until the next entry.
    // Use {time, dx, dy, dz} where displacement is in metres.
    //
    // Design principle: perturb while the robot is settled at a goal,
    // not during the transition to a new goal.
    //
    // Example below: one perturbation cycle per goal position.
    //   - Robot reaches goal at t = start_delay + i*intervalo
    //   - Perturbation starts 2s after goal arrival
    //   - Perturbation ends 4s later (2s before next goal)
    //
    /*std::vector<PerturbEvent> perturb_events = {
        // t=7s:  +5mm in X while at punto[0]
        {start_delay + 0*intervalo + 2.0,  0.05, 0.0, 0.0},
        // t=11s: perturbation OFF before next goal
        {start_delay + 0*intervalo + 6.0,  0.0,   0.0, 0.0},

        // t=17s: +5mm in Y while at punto[1]
        {start_delay + 1*intervalo + 2.0,  0.0,   0.05, 0.0},
        // t=21s: OFF
        {start_delay + 1*intervalo + 6.0,  0.0,   0.0,   0.0},

        // t=27s: +5mm in X and Y simultaneously while at punto[2]
        {start_delay + 2*intervalo + 2.0,  -0.05, -0.05, 0.0},
        // t=31s: OFF
        {start_delay + 2*intervalo + 6.0,  0.0,   0.0,   0.0},

        // t=37s: -5mm in X (opposite direction) while at punto[3]
        {start_delay + 3*intervalo + 2.0, -0.05, 0.0,   0.0},
        // t=41s: OFF
        {start_delay + 3*intervalo + 6.0,  0.0,   0.0,   0.0},

        // t=47s: +8mm in X (larger perturbation) while at punto[4]
        {start_delay + 4*intervalo + 2.0,  0.08, -0.08,   0.0},
        // t=51s: OFF
        {start_delay + 4*intervalo + 6.0,  0.0,   0.0,   0.0},

        // t=57s: no perturbation for punto[5] — baseline at the end
    };*/
    std::vector<PerturbEvent> perturb_events = {};  // lista vacía

    // ========================================================================
    // Loop
    // ========================================================================
    const ros::Time t_start = ros::Time::now();

    // Publish zero perturbation at startup so abdomenForceMock starts clean
    std_msgs::Float64MultiArray perturb_msg;
    perturb_msg.data = {0.0, 0.0, 0.0};
    pub_perturb.publish(perturb_msg);

    const double t_end = start_delay + puntos.size() * intervalo + 2.0;

    while (ros::ok()){
        const double elapsed = (ros::Time::now() - t_start).toSec();

        // ---- Fire goal events ----
        for (auto& ev : goal_events){
            if (!ev.sent && elapsed >= ev.time){
                pub_goal.publish(ev.p);
                ROS_INFO("Goal = [%.3f, %.3f, %.3f] at t=%.2f s",
                         ev.p.x, ev.p.y, ev.p.z, elapsed);
                ev.sent = true;
            }
        }

        for (auto& ev : goal_events_abdomen){
            if (!ev.sent && elapsed >= ev.time){
                pub_goal_abdomen.publish(ev.p);
                ROS_INFO("Goal abdomen = [%.3f, %.3f, %.3f] at t=%.2f s",
                         ev.p.x, ev.p.y, ev.p.z, elapsed);
                ev.sent = true;
            }
        }

        // ---- Fire perturbation events ----
        for (auto& ev : perturb_events){
            if (!ev.sent && elapsed >= ev.time){
                perturb_msg.data = {ev.dx, ev.dy, ev.dz};
                pub_perturb.publish(perturb_msg);
                /*ROS_INFO("Perturbation → [%.1f, %.1f, %.1f] mm at t=%.2f s",
                         ev.dx*1000, ev.dy*1000, ev.dz*1000, elapsed);*/
                ev.sent = true;
            }
        }

        if (elapsed >= t_end){
            ROS_INFO("Experiment finished at t=%.2f s", elapsed);
            ros::shutdown();
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}