#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import csv
import rospy
from std_msgs.msg import UInt8, String


class ExperimentStats(object):
    def __init__(self):
        # params
        self.output_csv = rospy.get_param("~output_csv", "/home/bsrl-ubuntu/new_ws/src/fm2_gather/data/exp_stats.csv")
        self.output_events = rospy.get_param("~output_events", "/home/bsrl-ubuntu/new_ws/src/fm2_gather/data/exp_replan_events.csv")
        self.trial_name = rospy.get_param("~trial_name", "")

        self.robot_ids = rospy.get_param("robot_ids", [1, 2, 3])
        self.num_robots = rospy.get_param("num_robots", len(self.robot_ids))

        # attempt to read replan params from monitor node if available
        self.replan_mode = rospy.get_param("/global_plan_monitor/replan_mode", "unknown")
        self.periodic_interval = rospy.get_param("/global_plan_monitor/periodic_interval", -1.0)

        # state
        self.run_active = False
        self.run_start_time = None
        self.last_active_start = None
        self.active_duration = 0.0
        self.run_end_time = None

        self.global_replans = 0
        self.per_robot_replans = {}
        self.aborted_counts = {}
        self.mv_states = {}
        for rid in self.robot_ids:
            self.per_robot_replans[rid] = 0
            self.aborted_counts[rid] = 0
            self.mv_states[rid] = None

        self.event_records = []

        # subs
        self.gather_start_sub = rospy.Subscriber("/gather_started", UInt8, self.gather_start_cb, queue_size=10)
        self.gather_signal_sub = rospy.Subscriber("/gather_signal", UInt8, self.gather_signal_cb, queue_size=10)
        self.replan_event_sub = rospy.Subscriber("/gather_replan_event", String, self.replan_event_cb, queue_size=10)

        for rid in self.robot_ids:
            topic = "/robot{}/mv_state".format(rid)
            rospy.Subscriber(topic, UInt8, self.mv_state_cb, callback_args=rid, queue_size=10)

        rospy.loginfo("ExperimentStats initialized. Output: %s", self.output_csv)

    def gather_start_cb(self, msg):
        if msg.data == 1 and not self.run_active:
            self.run_active = True
            self.run_start_time = rospy.Time.now()
            self.last_active_start = self.run_start_time
            rospy.loginfo("Experiment run started.")
        elif msg.data == 1 and self.run_active and self.last_active_start is None:
            self.last_active_start = rospy.Time.now()
            rospy.loginfo("Experiment run resumed.")
        elif msg.data == 0 and self.run_active:
            now = rospy.Time.now()
            if self.last_active_start is not None:
                self.active_duration += (now - self.last_active_start).to_sec()
                self.last_active_start = None
            # only finalize if all robots succeeded; otherwise treat as pause (e.g., replan)
            if self.all_succeeded():
                self.run_end_time = now
                self.run_active = False
                rospy.loginfo("Experiment run ended (all succeeded).")
                self.write_summary(status="completed")

    def gather_signal_cb(self, msg):
        if msg.data == 2:
            self.global_replans += 1
        elif msg.data >= 11:
            rid = msg.data - 10
            if rid not in self.per_robot_replans:
                self.per_robot_replans[rid] = 0
            self.per_robot_replans[rid] += 1

    def replan_event_cb(self, msg):
        ts = rospy.Time.now().to_sec()
        self.event_records.append((ts, msg.data))

    def mv_state_cb(self, msg, rid):
        self.mv_states[rid] = msg.data
        if msg.data == 5:
            if rid in self.aborted_counts:
                self.aborted_counts[rid] += 1

    def all_succeeded(self):
        for rid in self.robot_ids:
            if self.mv_states.get(rid, None) != 4:
                return False
        return True

    def write_summary(self, status):
        if self.run_start_time is None or self.run_end_time is None:
            return

        start_sec = self.run_start_time.to_sec()
        end_sec = self.run_end_time.to_sec()
        total_duration = end_sec - start_sec
        success = 1 if self.all_succeeded() else 0
        per_robot_total = sum(self.per_robot_replans.values())
        aborted_total = sum(self.aborted_counts.values())

        row = {
            "trial_name": self.trial_name,
            "start_time": start_sec,
            "end_time": end_sec,
            "total_duration_s": total_duration,
            "active_duration_s": self.active_duration,
            "success": success,
            "global_replans": self.global_replans,
            "per_robot_replans": per_robot_total,
            "aborted_total": aborted_total,
            "replan_mode": self.replan_mode,
            "periodic_interval": self.periodic_interval,
            "status": status,
        }

        write_header = not os.path.exists(self.output_csv)
        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)
        with open(self.output_csv, "a") as f:
            writer = csv.DictWriter(f, fieldnames=list(row.keys()))
            if write_header:
                writer.writeheader()
            writer.writerow(row)

        if self.event_records:
            write_event_header = not os.path.exists(self.output_events)
            os.makedirs(os.path.dirname(self.output_events), exist_ok=True)
            with open(self.output_events, "a") as f:
                w = csv.writer(f)
                if write_event_header:
                    w.writerow(["trial_name", "timestamp", "event"])
                for ts, event in self.event_records:
                    w.writerow([self.trial_name, ts, event])
            self.event_records = []

        rospy.loginfo("Experiment summary written to %s", self.output_csv)

    def shutdown(self):
        if self.run_active:
            now = rospy.Time.now()
            if self.last_active_start is not None:
                self.active_duration += (now - self.last_active_start).to_sec()
            self.run_end_time = now
            self.write_summary(status="interrupted")


if __name__ == "__main__":
    rospy.init_node("experiment_stats", anonymous=True)
    stats = ExperimentStats()
    rospy.on_shutdown(stats.shutdown)
    rospy.spin()
