"""
============================================================
open_challenge_navigator.py — WallFollower + CornerTurnController
একসাথে মিলিয়ে সম্পূর্ণ navigation সিদ্ধান্ত
============================================================
প্রতিটা LIDAR scan-এর জন্য একটাই ফাংশন কল করলেই servo angle পাওয়া
যাবে -- ভেতরে ভেতরে এটা বুঝে নেয় এখন wall-following করা উচিত নাকি
কোণ ঘোরা উচিত, caller-কে এই সিদ্ধান্ত নিজে নিতে হয় না।
============================================================
"""


class OpenChallengeNavigator:
    def __init__(self, wall_follower, corner_turn):
        self.wall_follower = wall_follower
        self.corner_turn = corner_turn

    def reset(self):
        self.wall_follower.reset()

    def compute_steering_angle(self, scan, current_yaw):
        """একটা LIDAR scan + বর্তমান yaw থেকে servo angle বের করে।
        state (wall-following/turning) ভেতরেই সামলানো হয়।"""
        if self.corner_turn.is_turning():
            angle = self.corner_turn.get_turn_angle()
            if self.corner_turn.check_turn_complete(current_yaw):
                # turn শেষ, নতুন straight section শুরু -- PID history
                # রিসেট করা হচ্ছে যাতে আগের সেকশনের জমে থাকা error
                # নতুন সেকশনে ভুল correction না দেয়
                self.wall_follower.reset()
            return angle

        if self.corner_turn.should_start_turn(scan):
            self.corner_turn.start_turn(scan, current_yaw)
            return self.corner_turn.get_turn_angle()

        return self.wall_follower.compute_steering_angle(scan)
